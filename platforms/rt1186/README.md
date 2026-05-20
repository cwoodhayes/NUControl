# rt1186 platform

CMake-based platform implementation for the FRDM-IMXRT1186 board (Cortex-M33 core).

Currently contains a blinky demo that toggles the user LED every second via SysTick.

## Prerequisites

- `arm-none-eabi-gcc` toolchain on your PATH
- The [RT1186](https://github.com/rohankotanu/RT1186) build project cloned alongside this repo
- `RT1186_DIR` environment variable pointing at the RT1186 project root:

```bash
# Add to ~/.zshrc
export RT1186_DIR=/path/to/RT1186
```

## Build

```bash
cmake --preset default
cmake --build --preset default
```

Output: `build/blinky_cm33.elf` and `build/blinky_cm33.bin`.

## Flash

```bash
LinkServer flash MIMXRT1186:FRDM-IMXRT1186 load build/blinky_cm33.elf
```
