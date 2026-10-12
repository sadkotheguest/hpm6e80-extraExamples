# pwmv2_cal_shared_delta_app

Standalone project for the **hpm6e00evk** board (SOC **HPM6E80**), packaging
`examples/pwmv2_cal_shared_delta_two_cmp_example.c`.

## What it does

Two PWMv2 CAL cells read the **same** shadow register `SHADOW_VAL[20]` as their
`S` input:

```
CAL0 = 1.0*DAC0 + S      -> CMP0 (rising)
CAL1 = 1.0*DAC1 + S      -> CMP1 (falling)
```

A circular HDMA transfer (PWM reload `pwm_dma_0` → TRGM `PWM1_REQ0` → DMAMUX →
HDMA ch0) writes **one word per PWM period** into `SHADOW_VAL[20]`, so **both**
compare edges shift by the same delta while the pulse width stays constant.
The DMA runs in `DMA_HANDSHAKE_OPT_ONE_BURST` mode for exactly that reason — see
"Two bugs fixed" below.

- Bases: `BASE0_COUNTS = 250`, `BASE1_COUNTS = 750`, period `reload = 1000`.
- **Output: PWM channel 0 → PE08 (`PWM1_P_0`), ~50% duty**, enabled in `pwm_setup()`.
- **Reference: PWM channel 1 → PE09 (`PWM1_P_1`)**, fixed 250/750 edges
  (CMP2/CMP3 fed from dedicated shadow registers, written once) — a *stationary*
  ~50% square wave to read the PE08 phase shift against on a scope.
  "Reference" is not a hardware mode here: it just means an output whose edges
  never move. Per UM §42.4.8 (printed p.797) *"Outputs 0 and 1 correspond to
  counter 0, compare values 0, 1, 2, 3"* — so channel 1 shares counter 0 and the
  period with channel 0, making the two directly comparable on a scope. With
  `PAIR_MODE=0` and `TRIG_SEL4=0`, outputs X and Y of a submodule are independent
  (X ← a/b = CMP0/CMP1, Y ← c/d = CMP2/CMP3; UM §42.4.5/§42.4.7, printed
  p.794/797), so channel 1 keeps its own edges. Do **not** move the reference to
  channel 2/3: those belong to counter 1, which this app does not run.
- Delta buffer lives in non-cacheable AHB SRAM (`.ahb_sram`), pre-shifted:
  `raw = count << 8`. Values step by 50 and sweep 0..200 counts (up to ~20% of
  the period); `BASE1 + max delta` must stay under `reload`, hence the cap.
- PWM instance: `BOARD_APP_PWM` (= `HPM_PWM1` on this board).

Pin muxing: `init_pwm_pins(PWM)` is called from `main()`. On this board it muxes
**PE08..PE13 as `PWM1_P_0..PWM1_P_5`** (`init_pwm1_pins()`, `board.c:1410`) and
only acts on `HPM_PWM1` — other instances are silently ignored. There is **no
`init_trgm_pins()`** on hpm6e00evk (the original example's comment was
copy-paste from another board); this design uses TRGM internally
(`PWM0_REQ0` → TRGM DMA0), so no external TRGM pin is required.

Real output is enabled: CMP0/CMP1 are routed to PWM channel 0 (PE08 = `PWM1_P_0`)
via `pwmv2_disable_four_cmp()` + `pwmv2_channel_enable_output()`, and the counter
is released with `pwmv2_start_pwm_output()`. Channel 1 (PE09 = `PWM1_P_1`) is
enabled the same way but its edges come from dedicated shadow registers that are
written once and never updated, so it stays fixed while PE08 shifts. On hardware
this yields two ~50% square waves on PE08/PE09 that share counter 0 and period:
PE08's edges track the DMA delta, PE09's do not. Recipe taken from the SDK
`pwmv2/pwm_output` sample (`generate_edge_aligned_waveform`), plus
`pwmv2_deinit()` for a known start state.

## Files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | SDK application project (`find_package(hpm-sdk)`) |
| `app.yaml` | SDK app metadata: declares the `pwmv2` dependency (capability guard, see below) |
| `src/pwmv2_cal_shared_delta_example.c` | Application source (based on `examples/`, with `init_pwm_pins(PWM)` enabled) |

### About `app.yaml`

Holds only `dependency: - pwmv2`. It does **not** enable the driver (that comes
from `soc/HPM6E00/HPM6E80/soc_modules.list`); its effect is that
`check_board_capability()` (`hpm_sdk/cmake/application.cmake:148` →
`scripts/check_board_cap.py`) fails loudly if the project is ever built for a
SoC/board without PWMv2. Verified passing for hpm6e00evk.

## Build

```bat
set HPM_SDK_BASE= [... sdk path ...]
set GNURISCV_TOOLCHAIN_PATH= [... toolchain path ...]
set PATH=%GNURISCV_TOOLCHAIN_PATH%\bin;D:\_SOFT\sdk_env_v1.13.0\tools\python3;D:\_SOFT\sdk_env_v1.13.0\tools\python3\Scripts;D:\_SOFT\sdk_env_v1.13.0\tools\cmake\bin;D:\_SOFT\sdk_env_v1.13.0\tools\ninja;%PATH%

cmake -G Ninja -DBOARD=hpm6e00evk -DHPM_BUILD_TYPE=flash_xip -DCMAKE_BUILD_TYPE=release -B build -S .
cmake --build build
```

Output lands in `build/output/`.

## References

- UM §42.4.9 (CAL calculate unit), §42.6.3 (SHADOW_VAL layout) — PWMv2.
- SDK: `drivers/inc/hpm_pwmv2_drv.h`, `hpm_dmav2_drv.h`, `hpm_trgm_drv.h`, `hpm_dmamux_drv.h`.
- Reference sample: `hpm_sdk/samples/drivers/pwmv2/pwm_output`.