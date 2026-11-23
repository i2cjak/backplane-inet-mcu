<!---
This file is used to generate the Tiny Tapeout project datasheet.
-->

## How it works

**TT_INet_MCU** is an autonomous Nordic-nRF and RP2040-PIO-inspired dual-tile **Interaction-Net Microcontroller** built for a Tiny Tapeout SKY130 `8x2` (16-tile) slot (`204,751.37 µm²`, `67.70%` raw utilization). Programs are compiled from a Vine-inspired functional/parallel source language (`.vine`) into Lafont/HVM-style interaction-net graph templates stored in external **QSPI XIP Flash** (`CS0_n`) and reduced across **two Chase-Lev work-stealing contexts** using latch-based hot storage backed seamlessly by external **QSPI PSRAM** (`CS1_n` for cold nodes/wires and Linked-List DMA descriptors/buffers, `CS2_n` for active-pair spill/fill).

Key architectural features:
- **24-bit Tagged Port ISA** (`TAG[3:0] | META[3:0] | VAL[15:0]`): supports `CON`, `DUP` (16 labels), `ERA`, `REF` (lazy QSPI XIP expansion + 8-entry REF dir cache + 16-entry per-node/wire hot relocation map), 16-bit `NUM`, two-phase `OPR`/`OP1` 16-bit ALU, `SWI` conditional branching, linear `VAR` substitution wires, and linear `IO` handles across devices `0..9`.
- **SKY130 `dlxtn_1` Latch-RF Hot Storage (`REG_INPUT=0`):** Stores `16×48b` hot nodes, `16×24b` hot wires, `2×4×48b` tile deques, `8×37b` REF cache, `16×10b` template relocation map, and `16×16b` PIO instruction memory using active-low SKY130 latches (`2,248 dlxtn_1`, `0 lpflow_inputisolatch`) with combinational RAW bypass where required.
- **Unified Reconfigurable `SERCOM` (`dev=0..2`):** Single shared shift register, baud/bit-rate divider, and 2-byte FIFO reconfigurable on the fly between full-duplex 8N1 **UART**, Mode 0–3 **SPI**, and open-drain **I2C** (with clock stretching), plus EasyDMA and PPI hooks.
- **Autonomous Real-Time Fabric (`dev=4..9`):**
  - **2×16-bit Timer/Counter/PWM (`inet_timer`, `dev=4`):** Compare/period events, hardware PWM outputs, and EasyDMA compare-update pacing.
  - **4-Channel PPI/DPPI (`inet_ppi`, `dev=5`):** Zero-CPU event-to-task crossbar (`12` events × `8` tasks) including `TASK_INET_INJECT` to spawn interaction-net pairs directly from hardware events.
  - **Universal Linked-List EasyDMA (`inet_dma`, `dev=6`):** Streams 4-word PSRAM descriptors across PSRAM A, `SERCOM`, `PIO`, `PWM0/1`, `Accelerator`, and `INet` pair injection.
  - **4×INT8 SIMD / 16×16 Q15 MAC Accelerator (`inet_accel`, `dev=7`):** Shared 9×9 signed multiplier executing 4-cycle dot-product MACs with hardware ReLU, arithmetic shift, and saturation.
  - **RP2040-Style PIO (`inet_pio`, `dev=8`):** Deterministic 1-cycle-per-instruction state machine with `16×16b` latch IMEM, 9-op RP2040 ISA (`JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET`), side-set/delay, `OSR`/`ISR`/`X`/`Y`, and DMA/PPI hooks.
  - **Nordic PSEL GPIO Crossbar (`inet_gpio_xbar`, `dev=9`):** Routes peripheral functions onto all non-QSPI/non-SWD pins (`uo[7:3]`, `uio[6:5]`, `ui[1,3,7]`) while keeping QSPI and SWD pins static/hardwired.
- **ARM ADIv5 SWD Debug Target:** Synchronously oversampled 2-wire SWD (`SWCLK`/`SWDIO`) supports halt, single-step, soft reset, telemetry counters, and full inspection/MMIO (`Region 0x7`) of all peripherals and memory.

### Pinout Summary (`ui[6]` selects pin mode; QSPI & SWD static, user pins PSEL-remappable)

- **`ui[6] = 0` (Full MCU Mode — all interfaces active):**
  - `ui`: `ui[0]=SWCLK` (static), `ui[1]=SPI_MISO` (xbar), `ui[2]=BOOT_HALT` (static), `ui[3]=UART_RX` (xbar), `ui[5:4]=QSPI_LAT[1:0]` (static), `ui[6]=QSPI_PMOD_MODE(0)`, `ui[7]=EXT_IRQ` (xbar)
  - `uo`: `uo[0]=QSPI_CS0_n` (static), `uo[1]=QSPI_CS1_n` (static), `uo[2]=QSPI_SCK` (static), `uo[3]=SPI_SCK` (xbar), `uo[4]=UART_TX` (xbar), `uo[5]=SPI_MOSI` (xbar), `uo[6]=SPI_CS_n` (xbar), `uo[7]=STATUS` (xbar)
  - `uio`: `uio[3:0]=QSPI_IO[3:0]` (static), `uio[4]=SWDIO` (static), `uio[5]=I2C_SCL` (xbar, default OD), `uio[6]=I2C_SDA` (xbar, default OD), `uio[7]=QSPI_CS2_n` (static)
- **`ui[6] = 1` (Drop-in `mole99` QSPI Pmod Mode):**
  - Remaps `uio[7:0]` to `{QSPI_CS2_n, QSPI_CS1_n, QSPI_IO[3], QSPI_IO[2], QSPI_SCK, QSPI_IO[1], QSPI_IO[0], QSPI_CS0_n}` while keeping UART, SPI, PWM, PIO, and XBAR on `uo`/`ui` active.

## How to test

1. **Compile a `.vine` program** into a QSPI Flash image using `compiler/` (e.g. `benchmarks/run_benchmarks.py` generates Flash `.hex`/`.bin` images for `smoke_arith`, `par_map_reduce`, `fib_rec`, `fsm_branch`, and `periph_io`).
2. **Program the QSPI Flash (`CS0_n`)** and assert `rst_n` low then high:
   - Leave `ui[2]` (`BOOT_HALT`) = `0` to boot and reduce immediately from Flash header `(boot_a, boot_b)`, monitoring UART output on `uo[4]` (`115200` baud at `24 MHz`, or programmable via `CFG_DIV_LO/HI`) and activity on `uo[7]` (`STATUS`).
   - Hold `ui[2]` (`BOOT_HALT`) = `1` out of reset to halt the core before execution and attach an SWD host (`SWCLK` on `ui[0]`, `SWDIO` on `uio[4]`) to read `DP IDCODE` (`0x11E70001`), configure `SERCOM`/`Timer/PWM`/`PPI`/`EasyDMA`/`Accel`/`PIO`/`XBAR` via Region `0x7` MMIO, inspect memory, or single-step reductions.
3. **Tune QSPI Read Latency:** Adjust `ui[5:4]` (`0..3` clock cycles) if running at higher `clk` frequencies to compensate for Tiny Tapeout mux/pad round-trip delay.
4. **Automated Simulation & Formal Verification:**
   - Formal spec & Z3 checks: `pytest -v spec/formal_checks.py compiler/test_compiler.py`
   - Cocotb RTL simulation: `make -C test SIM=verilator EXTRA_ARGS="--no-timing -Wno-STMTDLY"` (exercises QSPI Flash `W25Q128`, Dual QSPI PSRAM `APS6404L`, SWD, SERCOM UART/SPI/I2C, XBAR remapping, Timer/PWM + EasyDMA ramp, PPI `TASK_INET_INJECT`, RP2040 PIO 16×16 loopback, Chained DMA 4×INT8/Q15 Accel, and all 5 `.vine` benchmarks).

## External hardware

- **Tiny Tapeout QSPI Pmod (`mole99` or custom wiring):**
  - 1× QSPI NOR Flash (`W25Q128` or compatible, `0xEB` Quad Read) on `CS0_n`
  - 1–2× QSPI PSRAM (`APS6404L` or compatible, `0xEB`/`0x38`) on `CS1_n` (required for cold node/wire spills) and `CS2_n` (optional second bank for active-pair queue spills)
- **Tiny Tapeout Demoboard RP2040 UART1 (`ui[3]`/`uo[4]`):** USB-UART serial terminal (default 115200 8N1 at 24 MHz; configurable divider).
- **Optional External Peripherals (Mode 0, `ui[6]=0`):**
  - ARM SWD debugger probe (e.g. Picoprobe / CMSIS-DAP) on `ui[0]` (`SWCLK`) + `uio[4]` (`SWDIO`) with pull-up on `SWDIO`.
  - SPI slave device on `uo[3]` (`SCK`), `uo[5]` (`MOSI`), `ui[1]` (`MISO`), `uo[6]` (`CS_n`).
  - I2C slave device with external pull-up resistors (2.2–4.7 kΩ) on `uio[5]` (`SCL`) and `uio[6]` (`SDA`).
