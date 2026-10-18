# rgb_led_breathing_fade_app

Standalone project for the **hpm6e00evk** board (SOC **HPM6E80**), packaging
`examples/rgb_led_breathing_fade_hpm6e80.c`.

## What it does

Drives the on-board RGB LED with three independent PWMv2 channels and animates
them **simultaneously** — a smooth breathing envelope plus a continuous
cross-fade through a color palette. Unlike the stock SDK `rgb_led` sample, which
lights one color at a time in a blocking loop, all three channels update on
every animation tick.

| LED | Pin | PWM signal | Counter | Comparators | Shadow slots |
|---|---|---|---|---|---|
| Red | PE14 | `PWM1_P_6` | PWM1 cnt3 | CMP12 / CMP13 | 0, 1, 2 |
| Green | PE15 | `PWM1_P_7` | PWM1 cnt3 | CMP14 / CMP15 | 3, 4, 5 |
| Blue | PE04 | `PWM0_P_4` | PWM0 cnt2 | CMP8 / CMP9 | 6, 7, 8 |

Pin muxing comes from the board: `board_init_rgb_pwm_pins()` calls
`init_led_pins_as_pwm()` (`boards/hpm6e00evk/pinmux.c`), which assigns exactly
these three signals. `board_init_rgb_pwm_pins()` also turns the LED off first, so
no flash of a random color at boot.

## How it works

### Waveform generation

Per UM §42.4.5 (printed p.794 / PDF p.862), with `PAIR_MODE = 0` and
`TRIG_SEL4 = 0`, an even channel X is driven by compare values `a` and `b`:
with `a < b <= T`, the output is high for `a <= CNT < b`. Setting
`b = reload` and `a = duty` therefore gives **high fraction = (T - a)/T**.

The code uses `cmp2 = reload` (fixed) and `cmp1 = duty`, then inverts the pin
with `pwmv2_enable_output_invert()` when the board reports an active-high LED
(`board_get_led_pwm_off_level() == 0`). Net result: `cmp1 = 0` → LED fully off,
`cmp1 = reload` → LED fully on. The polarity decision is taken from the board
API, not hard-coded, so the example stays correct on an active-low LED.

### Glitch-free duty updates

`cmp1` takes its value from a shadow register and is latched on the counter
reload event:

```c
pwmv2_select_cmp_source(pwm, cmp1, cmp_value_from_shadow_val, shadow + 1);
pwmv2_cmp_update_trig_time(pwm, cmp1, pwm_shadow_register_update_on_reload);
```

Software can therefore write `SHADOW_VAL` at any point in the PWM period; the
new duty takes effect only at the period boundary, so a pulse is never truncated
mid-way. The initial values are pushed once with
`pwmv2_issue_shadow_register_lock_event()` (update trigger `on_shlk`), then the
trigger is switched to `on_reload` for runtime updates.

### Time base

The PWM1 counter-3 reload interrupt increments a millisecond counter:

```c
leds[0].reload = clock_get_frequency(clock_pwm1) / 1000 - 1;  /* 1 kHz carrier */
```

1 PWM period = 1 ms = 1 animation tick, so the animation rate is locked to the
PWM carrier and does not drift with CPU load. Blue runs on PWM0/counter 2 with
its own reload derived from `clock_pwm0`, which is the same nominal frequency —
visually the channels stay in sync.

### Animation math

1. **Color** — keyframes in `palette[]` are cross-faded with `smoothstep`
   (`3x² − 2x³`), which has zero slope at both ends and avoids a visible
   "kick" at color boundaries. `FADE_MS = 1200`.
2. **Breathing** — a triangle wave over `BREATH_MS = 2400` passed through
   `smoothstep`, with a floor `BREATH_MIN` so the LED does not fully extinguish
   (perceived as blinking rather than breathing).
3. **Gamma** — `level² / 255` (γ ≈ 2.0) approximates perceptual linearity
   without `powf()`; each channel's duty is then scaled to the counter reload.

## Tuning

| Constant | Meaning |
|---|---|
| `PWM_FREQ_HZ` | Carrier frequency (1 kHz default, no visible flicker) |
| `BREATH_MS` | Inhale + exhale duration |
| `BREATH_MIN` | Brightness floor, 0..1024 (0 = full off) |
| `FADE_MS` | Duration of one color → next color transition |
| `palette[]` | Color keyframes and their order |

For exact perceptual linearity replace the `v²` approximation in `gamma_duty()`
with a precomputed `powf(v/255, 2.2)` LUT.

## Files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | SDK application project (`find_package(hpm-sdk)`) |
| `app.yaml` | SDK app metadata: declares the `pwmv2` dependency |
| `src/main.c` | Application source |

## Build

```bat
set HPM_SDK_BASE= [... sdk path ...]
set GNURISCV_TOOLCHAIN_PATH= [... toolchain path ...]
set PATH=%GNURISCV_TOOLCHAIN_PATH%\bin;D:\_SOFT\sdk_env_v1.13.0\tools\python3;D:\_SOFT\sdk_env_v1.13.0\tools\python3\Scripts;D:\_SOFT\sdk_env_v1.13.0\tools\cmake\bin;D:\_SOFT\sdk_env_v1.13.0\tools\ninja;%PATH%

cmake -G Ninja -DBOARD=hpm6e00evk -DHPM_BUILD_TYPE=flash_xip -DCMAKE_BUILD_TYPE=release -B build -S .
cmake --build build
```

Output lands in `build/output/`.

## Notes / limitations

- Red and green share PWM1 counter 3, so they are phase-locked. The blue channel
  is independent (PWM0 counter 2). Irrelevant for an LED, but it rules out
  per-channel phase offsets.
- `g_ms` is 32-bit and wraps after ~49.7 days; the animation would jump once.
  Mask it or use `uint64_t` for long-running installations.
- `led_update()` runs from the main loop, not the ISR. A long blocking section
  elsewhere delays updates (the PWM output itself stays correct).
- The main loop busy-waits on `g_ms`; move `led_update()` into the ISR or use
  `WFI` if power matters.

## References

- UM §42.4.1 Shadow Registers (printed p.792 / PDF p.860)
- UM §42.4.3 Time Base (printed p.792 / PDF p.860)
- UM §42.4.5 PWM Generation (printed p.794 / PDF p.862)
- SDK: `drivers/inc/hpm_pwmv2_drv.h`, `samples/rgb_led/src/rgb_led.c`,
  `boards/hpm6e00evk/board.h`, `boards/hpm6e00evk/pinmux.c`
