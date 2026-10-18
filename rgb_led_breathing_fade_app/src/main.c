/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RGB LED: smooth breathing + continuous color transition (PWMv2)
 *
 * Target: HPM6E80 / hpm6e00evk
 *   PE14 -> PWM1_P_6 (red)    PWM1 counter 3, cmp 12/13
 *   PE15 -> PWM1_P_7 (green)  PWM1 counter 3, cmp 14/15
 *   PE04 -> PWM0_P_4 (blue)   PWM0 counter 2, cmp 8/9
 *
 * Design notes:
 *   - 1 kHz PWM carrier (no visible flicker), 1000 animation ticks per second.
 *   - Duty is written to a shadow register; PWMv2 latches it at the next reload,
 *     so there is never a torn/partial pulse.
 *   - Perceptual (gamma) correction is applied so the fade looks linear to the eye.
 *   - Breathing envelope and color cross-fade are both S-curved (smoothstep).
 */

#include <stdio.h>
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_pwmv2_drv.h"

#define PWM_FREQ_HZ      (1000U)   /* PWM carrier */
#define TICK_HZ          (1000U)   /* animation tick = PWM reload rate */

#define BREATH_MS        (2400U)   /* one full inhale/exhale */
#define BREATH_MIN       (128U)    /* brightness floor, 0..1024 (0 = full off) */
#define FADE_MS          (1200U)   /* time for one color -> next color */

typedef struct {
    uint8_t r, g, b;
} rgb_t;

/* Color keyframes walked in a loop; each transition is cross-faded. */
static const rgb_t palette[] = {
    { 255,   0,   0 },   /* red     */
    { 255, 128,   0 },   /* orange  */
    {   0, 255,   0 },   /* green   */
    {   0, 128, 255 },   /* cyan    */
    {   0,   0, 255 },   /* blue    */
    { 255,   0, 255 },   /* magenta */
};
#define PALETTE_N   ARRAY_SIZE(palette)

typedef struct {
    PWMV2_Type     *pwm;
    pwm_channel_t   ch;
    pwm_counter_t   cnt;
    uint8_t         shadow;   /* 3 consecutive slots: reload, cmp1, cmp2 */
    uint32_t        reload;
} led_ch_t;

static led_ch_t leds[3] = {
    { HPM_PWM1, pwm_channel_6, pwm_counter_3, 0, 0 },  /* red   */
    { HPM_PWM1, pwm_channel_7, pwm_counter_3, 3, 0 },  /* green */
    { HPM_PWM0, pwm_channel_4, pwm_counter_2, 6, 0 },  /* blue  */
};

static volatile uint32_t g_ms;   /* animation time base, advanced by PWM reload IRQ */

/* ------------------------------------------------------------------ */
/* PWM setup                                                          */
/* ------------------------------------------------------------------ */

static void led_pwm_init(void)
{
    uint8_t i;

    pwmv2_disable_shadow_lock_feature(HPM_PWM1);
    pwmv2_disable_shadow_lock_feature(HPM_PWM0);
    pwmv2_disable_counter(HPM_PWM1, pwm_counter_3);
    pwmv2_disable_counter(HPM_PWM0, pwm_counter_2);
    pwmv2_reset_counter(HPM_PWM1, pwm_counter_3);
    pwmv2_reset_counter(HPM_PWM0, pwm_counter_2);

    /* Shadow values: reload, cmp1 = 0 (0 % duty), cmp2 = reload */
    for (i = 0; i < ARRAY_SIZE(leds); i++) {
        pwmv2_set_shadow_val(leds[i].pwm, leds[i].shadow + 0, leds[i].reload, 0, false);
        pwmv2_set_shadow_val(leds[i].pwm, leds[i].shadow + 1, 0, 0, false);
        pwmv2_set_shadow_val(leds[i].pwm, leds[i].shadow + 2, leds[i].reload, 0, false);
    }

    /* Counter takes its reload from the first channel that uses it */
    pwmv2_counter_select_data_offset_from_shadow_value(HPM_PWM1, pwm_counter_3, leds[0].shadow);
    pwmv2_set_reload_update_time(HPM_PWM1, pwm_counter_3, pwm_reload_update_on_shlk);
    pwmv2_counter_burst_disable(HPM_PWM1, pwm_counter_3);

    pwmv2_counter_select_data_offset_from_shadow_value(HPM_PWM0, pwm_counter_2, leds[2].shadow);
    pwmv2_set_reload_update_time(HPM_PWM0, pwm_counter_2, pwm_reload_update_on_shlk);
    pwmv2_counter_burst_disable(HPM_PWM0, pwm_counter_2);

    /* Each channel: two comparators, edge-aligned */
    for (i = 0; i < ARRAY_SIZE(leds); i++) {
        uint8_t cmp1 = (uint8_t)(leds[i].ch * 2);
        uint8_t cmp2 = cmp1 + 1;

        pwmv2_select_cmp_source(leds[i].pwm, cmp1, cmp_value_from_shadow_val, leds[i].shadow + 1);
        pwmv2_cmp_update_trig_time(leds[i].pwm, cmp1, pwm_shadow_register_update_on_shlk);
        pwmv2_select_cmp_source(leds[i].pwm, cmp2, cmp_value_from_shadow_val, leds[i].shadow + 2);
        pwmv2_cmp_update_trig_time(leds[i].pwm, cmp2, pwm_shadow_register_update_on_shlk);

        pwmv2_disable_four_cmp(leds[i].pwm, HPM_NUM_TO_EVEN_FLOOR(leds[i].ch));

        /* LED is active high: cmp1 = 0 gives a low level, so invert the pin */
        if (!board_get_led_pwm_off_level()) {
            pwmv2_enable_output_invert(leds[i].pwm, leds[i].ch);
        }
        pwmv2_channel_enable_output(leds[i].pwm, leds[i].ch);
    }

    /* Push the initial shadow values into the work registers */
    pwmv2_issue_shadow_register_lock_event(HPM_PWM1);
    pwmv2_issue_shadow_register_lock_event(HPM_PWM0);

    /* Runtime duty updates: latch cmp1 at reload (no glitches) */
    for (i = 0; i < ARRAY_SIZE(leds); i++) {
        pwmv2_cmp_update_trig_time(leds[i].pwm, (uint8_t)(leds[i].ch * 2),
                                   pwm_shadow_register_update_on_reload);
    }

    pwmv2_enable_counter(HPM_PWM1, pwm_counter_3);
    pwmv2_start_pwm_output(HPM_PWM1, pwm_counter_3);
    pwmv2_enable_counter(HPM_PWM0, pwm_counter_2);
    pwmv2_start_pwm_output(HPM_PWM0, pwm_counter_2);
}

/* ------------------------------------------------------------------ */
/* Animation                                                          */
/* ------------------------------------------------------------------ */

SDK_DECLARE_EXT_ISR_M(IRQn_PWM1, pwm_tick_isr)
void pwm_tick_isr(void)
{
    uint32_t mask = 1u << pwm_counter_3;

    if (pwmv2_get_reload_irq_status(HPM_PWM1) & mask) {
        pwmv2_clear_reload_irq_status(HPM_PWM1, mask);
        g_ms++;
    }
}

/* x in 0..1024 -> 3x^2 - 2x^3, also 0..1024 */
static uint32_t smoothstep(uint32_t x)
{
    return (x * x * (3u * 1024u - 2u * x)) / (1024u * 1024u);
}

static uint8_t lerp8(uint8_t a, uint8_t b, uint32_t num, uint32_t den)
{
    return (uint8_t)((int32_t)a + (((int32_t)b - (int32_t)a) * (int32_t)num) / (int32_t)den);
}

/* 8-bit level -> compare value, with ~gamma 2.0 */
static uint32_t gamma_duty(uint32_t reload, uint32_t level8)
{
    uint32_t g = (level8 * level8 + 127u) / 255u;    /* 0..255 */
    return (uint32_t)(((uint64_t)reload * g + 127u) / 255u);
}

static void led_update(uint32_t t_ms)
{
    uint32_t phase = (t_ms / FADE_MS) % PALETTE_N;
    uint32_t frac  = ((t_ms % FADE_MS) * 1024u) / FADE_MS;
    uint32_t e     = smoothstep(frac);

    const rgb_t *c0 = &palette[phase];
    const rgb_t *c1 = &palette[(phase + 1u) % PALETTE_N];

    uint32_t r = lerp8(c0->r, c1->r, e, 1024);
    uint32_t g = lerp8(c0->g, c1->g, e, 1024);
    uint32_t b = lerp8(c0->b, c1->b, e, 1024);

    /* Breathing: triangle -> smoothstep, with a configurable floor */
    uint32_t bp = ((t_ms % BREATH_MS) * 2048u) / BREATH_MS;   /* 0..2047 */
    uint32_t tri = (bp < 1024u) ? bp : (2048u - bp);           /* 0..1024 */
    uint32_t breath = BREATH_MIN + (smoothstep(tri) * (1024u - BREATH_MIN)) / 1024u;

    uint32_t lr = (r * breath) / 1024u;
    uint32_t lg = (g * breath) / 1024u;
    uint32_t lb = (b * breath) / 1024u;

    pwmv2_set_shadow_val(leds[0].pwm, leds[0].shadow + 1, gamma_duty(leds[0].reload, lr), 0, false);
    pwmv2_set_shadow_val(leds[1].pwm, leds[1].shadow + 1, gamma_duty(leds[1].reload, lg), 0, false);
    pwmv2_set_shadow_val(leds[2].pwm, leds[2].shadow + 1, gamma_duty(leds[2].reload, lb), 0, false);
}

int main(void)
{
    uint32_t last = 0;

    board_init();
    board_init_rgb_pwm_pins();
    printf("rgb led breathing + color fade example\n");

    leds[0].reload = clock_get_frequency(clock_pwm1) / PWM_FREQ_HZ - 1u;
    leds[1].reload = leds[0].reload;                    /* red/green share counter 3 */
    leds[2].reload = clock_get_frequency(clock_pwm0) / PWM_FREQ_HZ - 1u;

    led_pwm_init();

    pwmv2_enable_reload_irq(HPM_PWM1, pwm_counter_3);
    intc_m_enable_irq_with_priority(IRQn_PWM1, 1);

    while (1) {
        uint32_t now = g_ms;

        if (now == last) {
            continue;               /* update at 1 kHz, driven by hardware */
        }
        last = now;
        led_update(now);
    }

    return 0;
}
