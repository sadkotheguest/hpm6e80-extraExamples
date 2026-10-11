# HPM6E80 Extra Examples

Additional peripheral examples for the **HPM6E80** (RISC-V) microcontroller on the
**hpm6e00evk** evaluation board, built on top of the official HPM SDK.

## Purpose

1. **EVK peripheral projects for the HPM6E80.**
   A collection of standalone example projects that exercise the on-chip
   peripherals of the HPM6E80 on the hpm6e00evk board.

2. **Debugging peripherals in modes not covered by the SDK.**
   The main idea is to target configurations and operating modes that are *not*
   represented in the stock HPM SDK samples — alternative clocking, DMA/trigger
   topologies, peripheral combinations, corner cases, and so on.

3. **LLM-assisted generation, hardware-verified.**
   Code generation may be assisted by LLMs, but **every example is debugged on
   real hardware**. Nothing here is shipped as "generated but untested": each
   project has been built and run on the actual EVK.

## Requirements

- HPM SDK (`hpm_sdk`) with `sdk_env`
- RISC-V toolchain bundled with the SDK
- hpm6e00evk board and debug probe

## Notes

- Examples follow the SDK layout/conventions and use SPDX headers.
- See the official HPM6E00 User Manual and Datasheet for register-level detail.

## Contacts

- Telegram: t.me/riscv_mcu
