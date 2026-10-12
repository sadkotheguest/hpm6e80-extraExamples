/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * HPM6E00 / PWMv2 — TWO CAL cells sharing ONE shadow register as their S input,
 * so a single DMA write moves BOTH compare edges (rising and falling) together.
 * ---------------------------------------------------------------------------
 * Requested shape:
 *      CAL0 = CMP0 + S
 *      CAL1 = CMP1 + S          <-- the SAME S, updated by DMA
 *
 * How "base + S" is built (UM §42.4.9, out = S + pT*T + pD*D):
 *      pT = 0                              (no period term)
 *      D  = DAC_VALUE<n>_SV,  pD = 8/8=1.0 (the fixed BASE, stored in a DAC reg)
 *      S  = SHADOW_VAL[20]                 (the shared, DMA-updated DELTA)
 *   => CAL0 = 1.0*DAC0 + S ;  CAL1 = 1.0*DAC1 + S
 *
 * Verified against the official SDK sample
 *   samples/drivers/pwmv2/pwm_output/src/pwm.c:450-504
 * which uses exactly: cmp_value_from_calculate + a DAC value + pD=8
 * (pwmv2_calculate_set_dac_value_parameter(...,8), pwmv2_set_dac_value(...),
 *  pwmv2_enable_software_dac_mode(...)).
 *
 * Key detail — the S input is chosen by CAL_CFG1.CAL_IN_OFF, per cell. Setting
 * BOTH cells to the same index (20) makes them read the SAME shadow register,
 * therefore one DMA word into SHADOW_VAL[20] shifts BOTH compare points by the
 * same delta; the pulse width (base1-base0) stays constant.
 *
 * IMPORTANT — raw shadow format for DMA:
 *   SHADOW_VAL[31:0] = count[24] | half_cycle(bit7) | hres_delay[7]   (UM §42.6.3)
 *   i.e. the count lives in bits [31:8]. The SDK's pwmv2_set_shadow_val() does
 *   (value << 8). A DMA source buffer must therefore hold PRE-SHIFTED words:
 *       buf[i] = delta_counts << 8
 *
 * Real output: CMP0/CMP1 drive PWM channel 0 (PE08 = PWM1_P_0). The pin driver
 * is enabled with pwmv2_channel_enable_output() and the counter released with
 * pwmv2_start_pwm_output(). Result: a ~50% square wave whose BOTH edges shift
 * together with S (phase moves, pulse width constant). Follows
 * samples/drivers/pwmv2/pwm_output (generate_edge_aligned_waveform /
 * generate_pwm_dac_output).
 *
 * Reference output: channel 1 (PE09 = PWM1_P_1) is a FIXED ~50% square wave.
 * Its edges (CMP2/CMP3) come from two dedicated shadow registers written once
 * and never updated, so it does NOT follow S. Both channels share counter 0 and
 * the same period, therefore the phase shift on PE08 is directly visible against
 * PE09 on a scope.
 *
 * Register refs:
 *   CAL[CFG0] 0x600+0x10n   CAL[CFG1] 0x604+0x10n
 *   CMP[CFG]  0x800+0x10n   SHADOW_VAL 0x008+0x4n
 *   DAC_VALUE_SV 0x2B0+0x4n GLB_CTRL2 0x1F4 (DAC_SW_MODE[27:24])
 *   CMP_IN_SEL[21:16], CMP_UPDATE_TIME[26:24]
 */

#include "board.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_dmav2_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"

#ifndef PWM
#define PWM BOARD_APP_PWM
#endif

/* ------------------------------------------------------------------ */
/* Configuration                                                       */
/* ------------------------------------------------------------------ */

/* The ONE shared S shadow register (delta), updated by DMA. */
#define DELTA_SHADOW        PWMV2_SHADOW_INDEX(20)

/* Two CAL cells, one per compare point. */
#define CAL_FOR_CMP0        PWMV2_CALCULATE_INDEX(0)
#define CAL_FOR_CMP1        PWMV2_CALCULATE_INDEX(1)

/* Base values live in DAC channels (software DAC mode).
 * FORMAT: the DAC feeds the CAL cell, whose output is used directly as a
 * compare point. Compare points and the counter both live in the SHADOW_VAL
 * format "count in bits [31:8]" (see UM §42.6.3), so the DAC value must be
 * pre-shifted by 8 as well. Writing a raw 250 makes CAL output 250, which the
 * comparator compares against the counter running in <<8 units -> the only
 * match is at count 0 -> a narrow spike instead of a square wave. */
#define BASE0_DAC           pwm_dac_channel_0
#define BASE1_DAC           pwm_dac_channel_1
#define BASE0_COUNTS        250U
#define BASE1_COUNTS        750U

/* pD = 1.0 -> code 8 (8/8). Range is +/-15/8. */
#define PD_ONE              ((uint8_t)8)

/* DMA channel used for the circular delta pump. */
#define DELTA_DMA_CH        (0U)

/* --- Reference output: fixed phase, for scope comparison ----------- */
/* "Reference" here is NOT a hardware mode: it just means an output whose edges
 * never move, so PE08's phase shift is visible against it on a scope.
 *
 * UM §42.4.8 (printed p.797): "Outputs 0 and 1 correspond to counter 0, compare
 * values 0, 1, 2, 3". So BOTH outputs 0 and 1 live on counter 0:
 *   target -> channel 0 -> PE08 (PWM1_P_0), CMP0 (rise) / CMP1 (fall)
 *   ref    -> channel 1 -> PE09 (PWM1_P_1), CMP2 (rise) / CMP3 (fall)
 * Channel 1 is output Y of submodule 0. When the submodule is NOT in pair mode
 * (PAIR_MODE=0) and TRIG_SEL4=0, outputs X and Y are independent: X depends on
 * a/b (CMP0/CMP1) and Y depends on c/d (CMP2/CMP3) — UM §42.4.5/§42.4.7,
 * printed p.794/797. So channel 1 is a valid independent reference and it shares
 * counter 0 with the target, which makes the two directly comparable.
 * NOTE: outputs 2/3 belong to a different submodule (counter 1), which this app
 * does not run — so the reference must NOT be moved to channel 2/3. */
#define REF_CH              pwm_channel_1
#define REF_CMP_RISE        PWMV2_CMP_INDEX(2)   /* channel 1 -> CMP2 (rising)  */
#define REF_CMP_FALL        PWMV2_CMP_INDEX(3)   /* channel 1 -> CMP3 (falling) */
#define REF_RISE_SHADOW     PWMV2_SHADOW_INDEX(1)  /* free: 0=period, 20=delta */
#define REF_FALL_SHADOW     PWMV2_SHADOW_INDEX(2)
#define REF_RISE_COUNTS     250U
#define REF_FALL_COUNTS     750U

/* Raw shadow word from a count value (count -> bits[31:8]). */
#define SHADOW_RAW(count)   ((uint32_t)((uint32_t)(count) << 8))
/* 24-bit two's complement of a positive count (for a negative delta). */
#define SHADOW_NEG24(v)     ((uint32_t)((0x1000000U - (uint32_t)(v)) & 0xFFFFFFU))

/* Circular delta buffer, non-cacheable AHB SRAM (HDMA reads it).
 * Values are PRE-SHIFTED raw shadow words. Step 50 counts, sweep 0..200, i.e.
 * up to ~20% of the 1000-count period -> clearly visible phase shift.
 * Constraint: CMP1 = BASE1(750) + S must stay inside the period (reload = 1000),
 * so S <= ~200 keeps a clean falling edge; larger values would lose it. */
#define DELTA_BUF_LEN       (8U)

/* The buffer is placed in AHB SRAM, which the linker script maps as a
 * (NOLOAD) section: its initialised values are NOT copied from flash at
 * startup. A static initialiser would therefore leave zeros in RAM (verified:
 * the DMA happily copied zeros into SHADOW_VAL20, so the phase never moved).
 * Hence the buffer is filled at runtime in main() before the DMA is started. */
static ATTR_PLACE_AT(".ahb_sram") uint32_t delta_buf[DELTA_BUF_LEN];

static const uint32_t delta_seq[DELTA_BUF_LEN] = {
    +50, +100, +150, +200, +150, +100, +50, 0,
};

static void delta_buf_init(void)
{
    for (uint32_t i = 0; i < DELTA_BUF_LEN; i++) {
        delta_buf[i] = SHADOW_RAW(delta_seq[i]);
    }
}

/* ------------------------------------------------------------------ */
/* 1) Two CAL cells, SAME S, base from a DAC register                  */
/* ------------------------------------------------------------------ */
static void cal_set_base_plus_shared_delta(uint8_t cal_index, uint8_t base_dac)
{
    /* out = S + pD*D   (pT = 0) */
    pwmv2_calculate_set_period_parameter   (PWM, cal_index, 0);        /* pT = 0    */
    pwmv2_calculate_set_dac_value_parameter(PWM, cal_index, PD_ONE);   /* pD = 1.0  */
    pwmv2_calculate_select_counter_calculate_index(PWM, cal_index, 0); /* T unused  */
    pwmv2_calculate_select_in_value(PWM, cal_index, base_dac);         /* D = DAC<x> */
    pwmv2_calculate_select_in_offset(PWM, cal_index, DELTA_SHADOW);    /* S = SHARED */
}

/* ------------------------------------------------------------------ */
/* 2) PWM / compare setup                                              */
/* ------------------------------------------------------------------ */
static void pwm_setup(void)
{
    uint32_t reload = 1000U;   /* period, matches BASE0/BASE1 scale */

    pwmv2_deinit(PWM);   /* start from a known state, as the pwm_output sample does */

    /* Shadow-register write enable (UM §42.6.1/§42.6.2, printed p.811, PDF 879).
     *
     * PWMv2 protection is TWO-STAGE, and BOTH stages are required:
     *   1) WORK_CTRL0.SHADOW_UNLOCK — "write 0x... first to unlock, then set
     *      related bits in unlock_sel". This is the only stage the SDK helper
     *      pwmv2_shadow_register_unlock() performs.
     *   2) UNLOCK (0x4) — UNLOCK_BIT[31:0], one bit per shadow register
     *      (bit2..bit29 = value_shadow, bit30 = force_mode): "the shadow
     *      registers can be updated only when related unlock_bit is set".
     *
     * The SDK never calls pwmv2_shadow_unlock_bit_mask(), so relying on
     * pwmv2_shadow_register_unlock() alone leaves every shadow register
     * write-protected: that is why the HDMA wrote to SHADOW_VAL[20] but the
     * register stayed 0 (SRCADDR kept sweeping while SHADOW20 never changed).
     *
     * We unlock ONCE here, for the shadow registers this app needs:
     *   SHADOW_VAL[0]  -> period        (unlock bit = 2 + 0  = 2)
     *   SHADOW_VAL[1]  -> ref rise      (unlock bit = 2 + 1  = 3)
     *   SHADOW_VAL[2]  -> ref fall      (unlock bit = 2 + 2  = 4)
     *   SHADOW_VAL[20] -> shared delta  (unlock bit = 2 + 20 = 22)
     * Once UNLOCK is set, the HDMA can write SHADOW_VAL[20] on every PWM
     * period with no further software involvement. */
    pwmv2_disable_shadow_lock_feature(PWM);
    pwmv2_shadow_register_unlock(PWM);                                    /* stage 1 */
    pwmv2_shadow_unlock_bit_mask(PWM,
                                 (1UL << (2U + 0U))  |   /* SHADOW_VAL[0]  period */
                                 (1UL << (2U + 1U))  |   /* SHADOW_VAL[1]  ref rise */
                                 (1UL << (2U + 2U))  |   /* SHADOW_VAL[2]  ref fall */
                                 (1UL << (2U + 20U)));   /* SHADOW_VAL[20] shared delta */

    /* Counter0 period */
    pwmv2_set_shadow_val(PWM, PWMV2_SHADOW_INDEX(0), reload, 0, 0);
    pwmv2_counter_select_data_offset_from_shadow_value(PWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));

    /* Shared delta starts at 0 */
    pwmv2_set_shadow_val(PWM, DELTA_SHADOW, 0, 0, 0);

    /* Bases 250 / 750 in DAC registers + software DAC mode.
     * The general-purpose calc-unit output feeds the CMP directly, and the CMP
     * compares against the counter, whose value lives in bits [31:8] (24-bit
     * integer + half cycle + hr_delay, UM §42.6.3). So the values entering the
     * calc unit must be PRE-SHIFTED by 8 to match: D = base<<8, S = delta<<8. */
    pwmv2_enable_software_dac_mode(PWM, BASE0_DAC);
    pwmv2_enable_software_dac_mode(PWM, BASE1_DAC);
    pwmv2_set_dac_value(PWM, BASE0_DAC, SHADOW_RAW(BASE0_COUNTS));
    pwmv2_set_dac_value(PWM, BASE1_DAC, SHADOW_RAW(BASE1_COUNTS));

    /* CAL0 = 1.0*DAC0 + S ; CAL1 = 1.0*DAC1 + S  (S is the SAME register) */
    cal_set_base_plus_shared_delta(CAL_FOR_CMP0, BASE0_DAC);
    cal_set_base_plus_shared_delta(CAL_FOR_CMP1, BASE1_DAC);

    /* Compare points come from the CAL cells */
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(0), cmp_value_from_calculate, 0); /* CAL0 -> CMP0 */
    pwmv2_select_cmp_source(PWM, PWMV2_CMP_INDEX(1), cmp_value_from_calculate, 1); /* CAL1 -> CMP1 */
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(0), pwm_shadow_register_update_on_reload);
    pwmv2_cmp_update_trig_time(PWM, PWMV2_CMP_INDEX(1), pwm_shadow_register_update_on_reload);

    /* Reference channel 1: FIXED edges from two dedicated shadow registers.
     * Written once here and never touched again (the DMA only writes
     * SHADOW_VAL[20]), so REF stays put while channel 0 shifts with S. */
    pwmv2_set_shadow_val(PWM, REF_RISE_SHADOW, REF_RISE_COUNTS, 0, 0);
    pwmv2_set_shadow_val(PWM, REF_FALL_SHADOW, REF_FALL_COUNTS, 0, 0);
    pwmv2_select_cmp_source(PWM, REF_CMP_RISE, cmp_value_from_shadow_val, REF_RISE_SHADOW);
    pwmv2_select_cmp_source(PWM, REF_CMP_FALL, cmp_value_from_shadow_val, REF_FALL_SHADOW);
    pwmv2_cmp_update_trig_time(PWM, REF_CMP_RISE, pwm_shadow_register_update_on_reload);
    pwmv2_cmp_update_trig_time(PWM, REF_CMP_FALL, pwm_shadow_register_update_on_reload);

    /* Counter behaviours (as in samples/drivers/pwmv2/phase_shifting & pwm_output):
     * reload from the shadow register, burst disabled. */
    pwmv2_set_reload_update_time(PWM, pwm_counter_0, pwm_reload_update_on_reload);
    pwmv2_counter_burst_disable(PWM, pwm_counter_0);

    /* NO shadow-lock dance: the shadow-lock feature is disabled, so
     * WORK_CTRL1.SHADOW_LOCK is always 0 and the shadow registers load to the
     * work registers straight through (UM §42.6.11: "if cleared, shadow_lock
     * will be always 0"). We only need the write-enable done above (UNLOCK
     * bits), which we leave set so the HDMA keeps writing SHADOW_VAL[20].
     * Do NOT call pwmv2_shadow_register_lock() here: it would set
     * WORK_CTRL1.SHADOW_LOCK and block subsequent shadow writes. */
    /* pwmv2_shadow_register_lock(PWM); */

    /* Route CMP0/CMP1 to PWM channel 0's output and drive the pin.
     * Two-cmp mode: CMP0 = rising edge, CMP1 = falling edge.
     * Channel 0 == PWM1_P_0 == PE08 (muxed by init_pwm_pins()). */
    pwmv2_disable_four_cmp(PWM, pwm_channel_0);
    pwmv2_channel_enable_output(PWM, pwm_channel_0);

    /* Reference channel 1 -> PE09 (PWM1_P_1), fixed phase.
     * Output Y of submodule 0: with PAIR_MODE=0 and TRIG_SEL4=0 it uses its own
     * CMP2/CMP3 (UM §42.4.5/§42.4.7, printed p.794/797) and shares counter 0
     * with channel 0, so the two outputs are directly comparable on a scope. */
    pwmv2_disable_four_cmp(PWM, REF_CH);
    pwmv2_channel_enable_output(PWM, REF_CH);

    pwmv2_reset_counter(PWM, pwm_counter_0);
    pwmv2_enable_counter(PWM, pwm_counter_0);
    pwmv2_start_pwm_output(PWM, pwm_counter_0);   /* SW start (CNT_GLBCFG.CNT_SW_START) */
}

/* ------------------------------------------------------------------ */
/* 3) Circular DMA writing the ONE shared S register                   */
/* ------------------------------------------------------------------ */
static void delta_dma_setup(void)
{
    /* PWM0 reload -> its own DMA request (counter0 -> pwm_dma_0) */
    pwmv2_enable_dma_at_reload_point(PWM, pwm_dma_0, pwm_counter_0);

    /* PWM1_REQ0 -> TRGM DMA0 -> DMAMUX -> HDMA channel.
     * NOTE: the PWM instance is HPM_PWM1 (BOARD_APP_PWM), so the TRGM source
     * must be HPM_TRGM0_DMA_SRC_PWM1_REQ0 (0x4), NOT the PWM0 variant (0x0). */
    trgm_dma_request_config(HPM_TRGM0, TRGM_DMACFG_0, HPM_TRGM0_DMA_SRC_PWM1_REQ0);

    dmamux_config(HPM_DMAMUX,
                  DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_HDMA, DELTA_DMA_CH),
                  HPM_DMA_SRC_MOT_0,        /* TRGM DMA0 */
                  true);

    dma_handshake_config_t cfg = {0};
    cfg.ch_index      = DELTA_DMA_CH;
    cfg.src           = (uint32_t)delta_buf;
    cfg.dst           = (uint32_t)&PWM->SHADOW_VAL[PWMV2_SHADOW_INDEX(20)]; /* the shared S */
    cfg.size_in_byte  = sizeof(delta_buf);
    cfg.data_width    = DMA_TRANSFER_WIDTH_WORD;
    cfg.src_fixed     = false;   /* advance through the buffer */
    cfg.dst_fixed     = true;    /* write the same register    */
    cfg.en_infiniteloop = true;  /* circular */
    cfg.interrupt_mask  = 0;
    dma_setup_handshake(HPM_HDMA, &cfg, true);

    /* Handshake option MUST be ONE_BURST, not ALL_TRANSIZE.
     *
     * UM §33: HANDSHAKEOPT 0 = "one request to transfer ONE burst",
     *         1 = "one request to transfer ALL the data defined in ch_tts".
     *
     * The peripheral request here is PWM counter0 reload -> pwm_dma_0, i.e. it
     * fires ONCE per PWM period. We want exactly ONE delta word latched into
     * SHADOW_VAL[20] per period — that is "one word per request".
     *
     * With ALL_TRANSIZE the DMA drained the whole 8-word ring inside ONE period
     * (SRCADDR swept f0200000..f020001c, TRANSIZE counted 8->0), so the last
     * word written each period was delta_buf[7] = 0: SHADOW_VAL20 read back 0
     * almost every time (only occasionally caught mid-sweep), and the CAL cell
     * output never moved -> both scope channels sat at a fixed ~50 %.
     *
     * With ONE_BURST (the dma_default_channel_config default, but set
     * explicitly here for clarity) each reload advances the ring by exactly one
     * word: SHADOW_VAL[20] = 50,100,150,200,150,100,50,0 over 8 periods, and
     * the CAL output (CMP_UPDATE_TIME = reload) latches one new compare point
     * per period. */
    dma_set_handshake_option(HPM_HDMA, DELTA_DMA_CH, DMA_HANDSHAKE_OPT_ONE_BURST);
}

int main(void)
{
     board_init();
    /* Mux the PWM1 outputs: PE08..PE13 -> PWM1_P_0..PWM1_P_5.
     * init_pwm_pins() is HPM_PWM1-only on this board (board.c:1410).
     * NOTE: there is no init_trgm_pins() on hpm6e00evk, and this design uses
     * TRGM internally (PWM0_REQ0 -> TRGM DMA0), so no external TRGM pin is needed. */
    init_pwm_pins(PWM);

    delta_buf_init();   /* must run before the DMA self-triggers (NOLOAD section) */
    pwm_setup();
    delta_dma_setup();

    for (;;) {
        /* Every PWM period the DMA writes the next word of delta_buf into
         * SHADOW_VAL[20]. BOTH CAL0 and CAL1 read that register as S, so
         * CMP0 (rising) and CMP1 (falling) shift together by the same delta,
         * while the pulse width (750 - 250) stays constant. */
    }
    return 0;
}

/*
 * Notes / caveats:
 *  - One CAL cell CANNOT drive two different compare points: both would read
 *    the same output value and collapse to one edge. Two cells are required;
 *    the "shared" part is the S register, not the cell.
 *  - Negative delta: write 24-bit two's complement into the buffer, e.g.
 *        SHADOW_RAW(0xFFFFF6)  ->  base + (-10)  (relies on modular add)
 *    If your silicon does not wrap, fall back to writing the final value.
 *  - Both CAL cells must use the SAME DELTA_SHADOW index (20) -> one write,
 *    two effects. Give them different indices and only one edge moves.
 *  - DAC registers must be in software mode (DAC_SW_MODE), else the DAC
 *    forwarder overrides them.
 *  - Delta buffer must be non-cacheable (.ahb_sram) or cache-flushed.
 *  - API names taken 1:1 from hpm_pwmv2_drv.h / hpm_trgm_drv.h / hpm_dmav2_drv.h.
 */