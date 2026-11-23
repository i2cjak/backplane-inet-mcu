# TT_INet_MCU Architecture & ISA Specification

**TT_INet_MCU** is an autonomous, Nordic-nRF and RP2040-PIO-inspired dual-tile Interaction-Net Microcontroller targeting a **Tiny Tapeout SKY130 `8x2` (16-tile)** slot (`tinytapeout-sky-26e`). Programs are compiled from Vine (`.vine`) into interaction-net graph templates stored in external QSPI XIP Flash (`CS0_n`), reduced across two Chase-Lev LIFO work-stealing contexts using SKY130 `dlxtn_1` latch-RF hot storage backed seamlessly by external QSPI PSRAM (`CS1_n` / `CS2_n`), with a unified reconfigurable `SERCOM`, autonomous PPI/DPPI Task/Event fabric, 2×16-bit Timer/Counter/PWM, Universal Linked-List EasyDMA, 4×INT8 SIMD / 16×16 Q15 MAC Accelerator, RP2040-style PIO, Nordic PSEL GPIO crossbar (on non-QSPI/SWD pins), and ARM ADIv5 SWD debug target.

---

## 1. Target Platform & SKY130 `8x2` Physical Budget

- **Geometry (`info.yaml` `tiles: "8x2"`):** `1378.16 × 225.76 µm` (~`311,133 µm²` die; `302,420 µm²` core area in [scripts/synth_sky130.sh](file:///home/h/Documents/i2cjak/TT_INet_MCU/scripts/synth_sky130.sh)).
- **Safe `8x2` Budget & Latch-RF Strategy:** Keeping total synthesized cell area `<= ~68%` raw (`~77–78%` est. post-CTS) preserves routability across `met1`–`met4`. Replacing DFF hot storage arrays with active-low `sky130_fd_sc_hd__dlxtn_1` latch RFs ([src/inet_latch_rf.v](file:///home/h/Documents/i2cjak/TT_INet_MCU/src/inet_latch_rf.v), `REG_INPUT=0`, `0 lpflow_inputisolatch`) doubles hot nodes/wires (`8→16`), doubles REF dir cache (`4→8`), adds a `16×10b` per-node/wire relocation map, and funds the entire autonomous peripheral fabric inside `67.70%` raw utilization.

### 1.1 Measured Subsystem Area Breakdown (`sky130_fd_sc_hd`, 50 MHz, `16N/16W/8D/2T`)

*(Hierarchical unflattened Yosys synthesis `/tmp/hier_final.log`; flat synthesized top is **204,751.37 µm²**, `67.70%` raw, `23,382` cells, `2,372` DFFs, `2,248` `dlxtn_1` latches, `54,548` GE.)*

| Subsystem / Module | Role | Cells | DFFs | `dlxtn_1` Latches | Area (`µm²`) | % of `8x2` Core |
|---|---|---:|---:|---:|---:|---:|
| `inet_core` (control + 2-tile Chase-Lev FSM) | Core reducer, QSPI/SWD/IO FSM | 7,969 | 887 | 0 | 62,840.27 | 20.78% |
| `u_hot_nodes` (`inet_latch_rf`, `16×48b`) | Hot node arena (`BYPASS=1`) | 1,341 | 0 | 768 | 15,618.73 | 5.16% |
| `u_dq_rf` (`inet_latch_rf`, `8×48b`) | 2×4-entry Chase-Lev deques (`BYPASS=1`) | 683 | 0 | 384 | 7,903.83 | 2.61% |
| `u_hot_wires` (`inet_latch_rf`, `16×24b`) | Hot linear wire arena (`BYPASS=1`) | 664 | 0 | 384 | 7,876.30 | 2.60% |
| `u_ref_dir` (`inet_latch_rf`, `8×37b`) | 8-entry Flash REF directory cache | 492 | 0 | 296 | 5,869.38 | 1.94% |
| `u_ref_map` (`inet_latch_rf`, `16×10b`) | Per-node/wire hot template remap | 302 | 0 | 160 | 3,360.72 | 1.11% |
| `inet_alu` | Shared 16-op combinational ALU | 1,081 | 0 | 0 | 6,536.27 | 2.16% |
| **INet Compute Core Subtotal** | **16N/16W/8D/2T + ALU + Latch RFs** | **12,532** | **887** | **1,992** | **110,005.50** | **36.37%** |
| `inet_pio` (excl. IMEM) + `u_imem` (`16×16b`) | RP2040 9-op PIO SM (`256` latches) | 2,465 | 203 | 256 | 20,611.02 | 6.82% |
| `inet_sercom` | Unified UART/SPI/I2C (`2B` FIFO) | 1,581 | 171 | 0 | 12,421.91 | 4.11% |
| `inet_accel` | Shared 9×9 4×INT8 SIMD / Q15 MAC | 1,449 | 113 | 0 | 10,963.01 | 3.62% |
| `inet_io_bridge` | Linear IO + SERCOM/fast/SWD arbiter | 900 | 214 | 0 | 9,900.75 | 3.27% |
| `inet_dma` | Linked-List EasyDMA engine | 952 | 188 | 0 | 9,328.95 | 3.08% |
| `inet_swd` | ARM ADIv5 SWD debug target | 888 | 175 | 0 | 9,028.66 | 2.99% |
| `inet_timer` | 2×16-bit Timer/Counter/PWM | 808 | 130 | 0 | 7,414.61 | 2.45% |
| `inet_qspi_ctrl` | Shared Flash/PSRAM A/B QSPI ctrl | 671 | 124 | 0 | 5,929.44 | 1.96% |
| `inet_ppi` | 4-ch PPI crossbar + INet injector | 340 | 104 | 0 | 4,604.42 | 1.52% |
| `inet_gpio_xbar` | Nordic PSEL GPIO crossbar (user pins) | 409 | 61 | 0 | 3,976.31 | 1.31% |
| `tt_um_inet_mcu` | Top-level QSPI/DMA arbiter & pin mux | 163 | 2 | 0 | 1,680.36 | 0.56% |
| **Flat Synthesized Top (`scripts/synth_sky130.sh`)** | **`tt_um_inet_mcu` (flattened + ABC opt)** | **23,382** | **2,372** | **2,248** | **204,751.37** | **67.70%** |

### 1.2 Tile & Hot-Storage Feasibility Sweep (`sky130_fd_sc_hd`, `8x2`)

Because `inet_core` shares a single reduction pipeline and a single external QSPI bus (`1-4-4` bottleneck), scaling `HOT_NODES/HOT_WIRES` from `8` to `16` cuts cycles/QSPI traffic more than adding a 3rd deque context while keeping raw area below 68%:

| Configuration (`N` / `W` / `D` / `T`) | DFFs | `dlxtn_1` Latches | Total Area (`µm²`) | `8x2` Raw Util | `8x2` Fit Verdict |
|---|---:|---:|---:|---:|---|
| **`16N / 16W / 8D / 2T` (Locked)** | **2,372** | **2,248** | **204,751.37** | **67.70%** | **SWEET SPOT (`<= 68%`, best IPC + full 16N/16W)** |
| `12N / 12W / 8D / 2T` | 2,248 | 1,960 | 200,210.77 | 66.20% | Extra routing margin, slightly smaller hot set |
| `16N / 12W / 8D / 2T` | 2,253 | 2,152 | 205,233.08 | 67.86% | Tight wire arena for REF templates |
| `12N / 12W / 8D / 3T` | 2,257 | 2,344 | 208,453.67 | 68.93% | Exceeds `68%` raw (`~79.3%` post-CTS) |
| `16N / 16W / 8D / 3T` | 2,267 | 2,632 | 215,296.49 | 71.19% | Too tight (`~81.9%` post-CTS) |
| `24N / 16W / 8D / 2T` | 2,271 | 2,664 | 216,216.12 | 71.49% | Too tight (`~82.2%` post-CTS) |

---

## 2. Dual-Mode 24-Pin Assignment & Nordic PSEL GPIO Crossbar

QSPI (`CS0/1/2`, `SCK`, `IO[3:0]`, `QSPI_LAT[1:0]`, `QSPI_PMOD_MODE`), `BOOT_HALT`, and SWD (`SWCLK`, `SWDIO`) remain **static/hardwired** for clean QSPI timing and uninterrupted debug. All remaining user pins (`ui[1,3,7]`, `uo[7:3]`, `uio[6:5]`) route through `inet_gpio_xbar` (`dev=9`) with safe reset defaults matching the original fixed pinout:

### Mode 0 (`ui[6] = 0`): Full MCU Mode (Reset Defaults + Crossbar Capability)

| Bank | Pin | Reset Default Signal | Dir | XBAR Selectable | Description |
|---|---|---|---|---|---|
| `ui` | `ui[0]` | `SWCLK` | In | Static | ARM SWD clock (synchronously oversampled in `clk`) |
| `ui` | `ui[1]` | `SPI_MISO` | In | `in_sel1` | User SPI controller MISO (or any mapped peripheral input) |
| `ui` | `ui[2]` | `BOOT_HALT` | In | Static | `1` = halt core at reset for SWD inspection/loading |
| `ui` | `ui[3]` | `UART_RX` | In | `in_sel0` | UART RX (matches Tiny Tapeout demoboard RP2040 UART1) |
| `ui` | `ui[5:4]` | `QSPI_LAT[1:0]` | In | Static | Programmable QSPI read capture delay (`0..3` `clk` cycles) |
| `ui` | `ui[6]` | `QSPI_PMOD_MODE` | In | Static | `0` = Full MCU mode, `1` = `mole99` QSPI Pmod mode |
| `ui` | `ui[7]` | `EXT_IRQ` | In | `in_sel6/7` | External IRQ / Timer capture / PPI GPIO edge input |
| `uo` | `uo[0]` | `QSPI_CS0_n` | Out | Static | QSPI Flash XIP chip select (active low) |
| `uo` | `uo[1]` | `QSPI_CS1_n` | Out | Static | QSPI PSRAM Bank A chip select (Node/Wire Arena + DMA) |
| `uo` | `uo[2]` | `QSPI_SCK` | Out | Static | Shared QSPI serial clock |
| `uo` | `uo[3]` | `SPI_SCK` (`out_sel0`) | Out | `out_sel0` | Remappable (`UART_TX`, `SPI_*`, `PWM0/1`, `PIO[3:0]`, `STATUS`, `GPIO`) |
| `uo` | `uo[4]` | `UART_TX` (`out_sel1`) | Out | `out_sel1` | Remappable; defaults to `FN_UART_TX(1)` (RP2040 UART1) |
| `uo` | `uo[5]` | `SPI_MOSI` (`out_sel2`) | Out | `out_sel2` | Remappable; defaults to `FN_SPI_MOS(3)` |
| `uo` | `uo[6]` | `SPI_CS_n` (`out_sel3`) | Out | `out_sel3` | Remappable; defaults to `FN_SPI_CS(4)` |
| `uo` | `uo[7]` | `STATUS` (`out_sel4`) | Out | `out_sel4` | Remappable; defaults to `FN_STATUS(13)` (`core_busy | status_led`) |
| `uio` | `uio[3:0]` | `QSPI_IO[3:0]` | Bidir | Static | Shared QSPI quad data lines |
| `uio` | `uio[4]` | `SWDIO` | Bidir | Static | ARM SWD bidirectional data |
| `uio` | `uio[5]` | `I2C_SCL` (`out_sel5`) | Bidir/OD | `out_sel5` | Remappable; defaults to `FN_I2C_SCL(5)` (open-drain) |
| `uio` | `uio[6]` | `I2C_SDA` (`out_sel6`) | Bidir/OD | `out_sel6` | Remappable; defaults to `FN_I2C_SDA(6)` (open-drain) |
| `uio` | `uio[7]` | `QSPI_CS2_n` | Out | Static | QSPI PSRAM Bank B chip select (active low) |

### Mode 1 (`ui[6] = 1`): Drop-In `mole99` QSPI Pmod Mode

Remaps `uio[7:0]` to `{QSPI_CS2_n, QSPI_CS1_n, QSPI_IO[3], QSPI_IO[2], QSPI_SCK, QSPI_IO[1], QSPI_IO[0], QSPI_CS0_n}` while `SERCOM` UART/SPI, Timer/PWM, PIO, and XBAR outputs on `uo[7:3]`/`ui[1,3,7]` stay active. (`SWD` and `I2C` on `uio` park idle.)

---

## 3. Interaction-Net ISA (Synced with `spec/isa.py`)

### 3.1 24-Bit Tagged Port Encoding

Every port is 24 bits on-chip and stored little-endian, 4-byte aligned in Flash XIP and PSRAM:

```
  23      20 19      16 15                                     0
 +----------+----------+----------------------------------------+
 | TAG[3:0] | META[3:0]|               VAL[15:0]                |
 +----------+----------+----------------------------------------+
```

- `TAG[3] == 0` — **Atoms, Wires, and Sinks** (no heap node body):
  - `0x0` `FREE`: Uninitialized / empty wire slot.
  - `0x1` `VAR`: 1-port linear substitution wire; `VAL[14:0]` selects wire slot `0..32767`.
  - `0x2` `ERA`: Eraser (0-ary sink).
  - `0x3` `NUM`: 16-bit integer literal `VAL[15:0]`.
  - `0x4` `REF`: Immutable QSPI Flash XIP template index `VAL[15:0]` (lazy expansion).
  - `0x5` `ROOT`: Output root sink `VAL[15:0]` (root index `0..3` in hardware).
- `TAG[3] == 1` — **2-Port Heap Nodes** (`VAL[14:0]` = node ID `0..32767`, each node holds `{aux0, aux1}`):
  - `0x8` `CON`: Constructor / pair node `(aux0, aux1)`.
  - `0x9` `DUP`: Duplicator with 4-bit label `META[3:0]` (`0..15`), outputs `(aux0, aux1)`.
  - `0xA` `OPR`: Binary ALU node waiting for LHS `NUM` (`META` = `AluOp`, `aux0` = RHS, `aux1` = output port).
  - `0xB` `OP1`: Partially applied ALU node (`META` = `AluOp`, `aux0` = LHS `NUM`, `aux1` = output port).
  - `0xC` `SWI`: Conditional branch on zero/nonzero (`META[0]` = `SwiMode`: `0=DEC`, `1=RAW`; `aux0` = branches `CON(z, nz)`, `aux1` = output port).
  - `0xD` `IO`: Linear peripheral handle (`META` = `IoDev`: `0=UART, 1=SPI, 2=I2C, 3=SYS`; `aux0` = continuation port).

### 3.2 ALU, Switch, and IO Encodings

- **`AluOp` (`META[3:0]` on `OPR`/`OP1`):** `0=ADD`, `1=SUB`, `2=AND`, `3=OR`, `4=XOR`, `5=SHL`, `6=SHR` (logical), `7=SAR` (arithmetic), `8=EQ`, `9=NE`, `10=LTU`, `11=LTS`, `12=LEU`, `13=MUL` (low 16b), `14=SEQ` (returns `b` after forcing both operands), `15=PACK` (`{(a&0xFF), (b&0xFF)}`).
- **`SwiMode` (`META` on `SWI`):**
  - `0=DEC`: if `n == 0` links `br ~ CON(out, ERA)`; else allocates `c2 = CON(NUM(n-1), out)` and links `br ~ CON(ERA, c2)`.
  - `1=RAW`: same, except nonzero branch passes `NUM(n)` unmodified.
- **`NUM >< IO` Linear Peripheral Protocol:** `VAL` of `NUM` encodes `{4'h0, op[3:0], data[7:0]}`. Upon completion, `io_bridge` emits `cont ~ NUM({status[7:0], rdata[7:0]})`. Duplicating (`DUP >< IO`) or erasing (`ERA >< IO`) a linear `IO` handle is statically forbidden and raises a runtime linearity fault if encountered.
  - `UART (dev=0)`: `0=TX`, `1=RX_BLOCK`, `2=RX_POLL`, `3=STATUS`, `4=TX_WAIT`, `5=CFG_DIV_LO`, `6=CFG_DIV_HI`, `7=FIFO_PUSH`.
  - `SPI (dev=1)`: `0=XFER`, `1=CS_LOW`, `2=CS_HIGH`, `3=STATUS`, `4=CFG_DIV`, `5=CFG_MODE`.
  - `I2C (dev=2)`: `0=START`, `1=STOP`, `2=WRITE_BYTE`, `3=READ_ACK`, `4=READ_NACK`, `5=STATUS`, `6=CFG_DIV`. (`F=OP_SERCOM_MODE` switches `UART/SPI/I2C` explicitly without auto-switch.)
  - `SYS (dev=3)`: `0=NOP`, `1=CYCLE_LO`, `2=CYCLE_HI`, `3=GPIO_RD` (`ext_irq`), `4=STATUS_LED`.
  - `TIMER / PWM (dev=4)`: `[3]=0/1` selects `Timer0`/`Timer1`; `0=CFG` (`[6:4] presc, [3] pwm_pol, [2] cnt_mode, [1] auto_reload, [0] en`), `1=CC_LO`, `2=CC_HI`, `3=TOP_LO`, `4=TOP_HI`, `5=READ_CNT`, `6=CTRL` (`[0] start+clr, [1] stop, [2] clr_sticky`), `7=STATUS`. Compare & period events drive PPI and EasyDMA PWM pacing.
  - `PPI / DPPI (dev=5)`: `0..3=CH0..3` (`[7] en, [6:4] task, [3:0] evt`), `4..6=INJ_A[23:0]`, `7..9=INJ_B[23:0]`, `10=SW_TASK_TRIG`, `11=EVTS_RD` (`write [0]=1` clears sticky events). Events: `0=T0_CMP, 1=T0_PER, 2=T1_CMP, 3=T1_PER, 4=SC_DONE, 5=SC_RX, 6=EXT_IRQ, 7=DMA_DESC_DONE, 8=DMA_CHAIN_DONE, 9=PIO_IRQ, 10=PIO_RX_NE, 11=GPIO_EDGE`. Tasks: `0=T0_START, 1=T0_STOP, 2=T1_COUNT, 3=T1_START, 4=DMA_START, 5=SC_KICK, 6=PIO_START, 7=INET_INJECT`.
  - `EASYDMA (dev=6)`: `0..2=DESC_PTR[19:0]`, `3=START`, `4=ABORT`, `5=STATUS` (`[9] done_sticky, [8] busy, [5:0] desc_count`), `6=DESC_COUNT`. 4-word descriptor in PSRAM A (`src_addr, dst_addr, {flags[11:0], len[11:0]}, next_ptr`), `src`: `0=PSRAM, 1=SERCOM, 2=PIO, 3=ACCEL, 4=T0_CNT, 5=T1_CNT, 6=ZERO`; `dst`: `0=PSRAM, 1=SERCOM, 2=PIO, 3=PWM0, 4=PWM1, 5=ACCEL_AB, 6=ACCEL_ONE, 7=INET_PAIR`.
  - `ACCEL (dev=7)`: `0=CFG` (`[7:3] shift(0,2,4,6,7,8,12,15), [2] auto_clr, [1] relu, [0] mode_q15`), `1..4=OP_A[31:0]`, `5..8=OP_B[31:0]` (`8` starts MAC), `9=RUN`, `10/11=ACC_LO/HI`, `12=READ_QUANT`, `13=CLEAR`, `14=STATUS`. Shared `9×9` signed multiplier runs 4×INT8 SIMD (`4` cycles) or 16×16 Q15 MAC (`4` cycles).
  - `PIO (dev=8)`: `0=CFG` (`[7] sm_en, [6] autopull, [5] autopush, [4] shift_r_out, [3] shift_r_in, [2] sideset_en, [1] clr, [0] rst_sm`), `1=SET_ADDR`, `2/3=IMEM_LO/HI` (writes `16×16` latch IMEM), `4=SET_WRAP`, `5/6=SET_DIV`, `7/8=TX_LO/HI`, `9=RX_POP`, `10=STATUS`, `11=IRQ_CLR`, `12=READ_REG`.
  - `GPIO XBAR (dev=9)`: `0..6=OUT_SEL0..6` (`uo[3..7]`, `uio[5..6]` -> function `0..15`), `7=IN_SEL_CFG` (`{slot[2:0], src[2:0]}` + `[7]=0`), `8=IN_SEL_CFG_2` (`in_sel6/7`), `9=GPIO_OUT`, `10=GPIO_OE`.

*(Note: Unified Region `0x7` SWD MMIO maps `addr = 0x70_0000 | (dev << 8) | (op << 2)` to every device `0..9` identically to linear `NUM >< IO`.)*

### 3.3 Core Reduction & Linking Rules

`link(a, b)` resolves substitution wires before enqueueing an active pair:
1. If either port is `VAR(w)`: atomically inspect `wire[w]`. If `FREE`, store the opposite port into `wire[w]` and finish; otherwise free `w` and continue linking with the stored port (handling `VAR-VAR` chains cleanly).
2. If either port is `ROOT(r)` and the other is not `REF`, latch the result into `ROOT[r]`.
3. Otherwise push `(a, b)` as an active pair onto the tile's local queue.

Active-pair rewrite rules:
- `CON ⋈ CON`: free both nodes, `link(a0, b0)`, `link(a1, b1)`.
- `DUP(L) ⋈ DUP(L)` (same label): annihilate — free both, `link(d0, b0)`, `link(d1, b1)`.
- `DUP(L) ⋈ X` (`X` in `{CON, DUP(M!=L), OPR, SWI}`): commute — free both, allocate 4 nodes + 4 wires, cross-connect.
- `DUP(L) ⋈ OP1(op, lhs, out)`: commute specialized 1-node `OP1` — allocate 3 nodes (`op_a, op_b, nd`) + 2 wires (`w0, w1`).
- `DUP(L) ⋈ {NUM, REF}`: O(1) copy — free `DUP`, `link(d0, atom)`, `link(d1, atom)`.
- `ERA ⋈ {ERA, NUM, REF}`: discard in O(1).
- `ERA ⋈ {CON, DUP, OPR, OP1, SWI}`: free node, propagate `ERA` to connected ports (`OP1` only to `out`).
- `REF(id) ⋈ X` (`X != DUP`): lazy template expansion from QSPI Flash XIP (`CS0_n`), allocating fresh node/wire IDs, relocating template ports, linking `root_port ~ X` and internal redexes.
- `NUM ⋈ OPR`: reuse same node slot in-place (`write_node(vb, NUM, out)`), `link(rhs, OP1(op, vb))`.
- `NUM ⋈ OP1`: free `OP1` node, compute `eval_alu(op, lhs, rhs)`, `link(out, NUM(res))`.
- `NUM ⋈ SWI`: zero/nonzero branch rewrite reusing `vb` as outer `CON`.
- `NUM ⋈ IO`: dispatch to `inet_io_bridge` continuation slot; tile remains free for other work.

---

## 4. Two-Tile Chase-Lev LIFO + Zero-Coherency PSRAM Microarchitecture

### 4.1 Why N=2 Tiles + LIFO Scheduling

1. **Area Fit:** Two reduction tiles share one QSPI bus arbiter (`inet_qspi_ctrl`), one IO bridge (`inet_io_bridge`), and one SWD debug port (`inet_swd`), fitting comfortably inside the ~100–130k µm² core budget with healthy routing margin.
2. **Co-Design Result (LIFO vs FIFO):** In `benchmarks/BENCHMARK_REPORT.md`, pure FIFO breadth-first execution expands `par_map_reduce` to **247 live nodes / 300 wires** and `fib_rec` to **101 live nodes / 267 wires**, causing heavy PSRAM spill traffic. **Per-tile LIFO local pop + oldest-end peer stealing** slashes peak live nodes **2.0–3.5×** (`par_map_reduce`: 247→73, `fib_rec`: 101→29, `fsm_branch`: 108→53, `periph_io`: 68→38).
3. **Latency Hiding:** While Tile 0 waits on a QSPI Flash/PSRAM burst or a slow UART/I2C peripheral transaction, Tile 1 continues executing hot reductions in parallel.

### 4.2 Zero-Coherency Hot-ID Split (On-Chip FFs + PSRAM A/B)

Because every interaction-net wire is strictly linear (single owner, no aliasing, no shared-memory mutation), partitioning the node and wire namespaces by ID threshold requires **zero CAM lookup, zero write-back eviction, and zero cache coherency protocol**:

- **Hot On-Chip Nodes (`nid < HOT_NODES`, synthesized `HOT_NODES=16` × 48 bits):** Single-cycle read/write in `dlxtn_1` latch RF (`u_hot_nodes`, `BYPASS=1`); priority allocator prefers lowest free hot ID first.
- **Cold PSRAM A Nodes (`nid >= HOT_NODES`):** Stored in **PSRAM Bank A (`CS1_n`)** at byte address `0x00_0000 + (nid << 3)` (`+0` = `aux0`, `+4` = `aux1`, 8 bytes/node).
- **Hot On-Chip Wires (`wid < HOT_WIRES`, synthesized `HOT_WIRES=16` × 24 bits):** Single-cycle atomic test-and-swap during `link()` via `dlxtn_1` latch RF (`u_hot_wires`, `BYPASS=1`).
- **Cold PSRAM A Wires (`wid >= HOT_WIRES`):** Stored in **PSRAM Bank A (`CS1_n`)** at byte address `0x04_0000 + (wid << 2)` (4 bytes/wire).
- **Per-Tile Active-Pair Deques + PSRAM B Spill Ring:** Two Chase-Lev 4-entry × 48-bit LIFO deques stored in shared `8×48b` latch RF (`u_dq_rf`, `BYPASS=1`). When both deques fill, pairs spill to a circular ring in **PSRAM Bank B (`CS2_n`)** at `0x00_0000 + (idx << 3)` (8 bytes/pair) and refill on underflow. A tile whose local deque is empty steals from the **oldest** end of its peer tile's deque before touching PSRAM B.
- **Flash XIP Template Expander (`CS0_n`):** Streams `1 + 2*num_nodes + 2*num_redex` words via `inet_qspi_ctrl` (`0xEB`), relocating node/wire IDs into free **hot slots first** (using a `16×10b` latch map `u_ref_map`) with cold fallback to PSRAM A, backed by an `8×37b` REF directory cache (`u_ref_dir`, cutting recursive `fib_rec` PSRAM spills **12.8×** from `77→6` and cycles **43.1%**).

---

## 5. Memory Maps & ARM ADIv5 SWD Debug Space

### 5.1 QSPI Flash XIP (`CS0_n`) Image Layout

All words are 32-bit little-endian; 24-bit ports occupy bits `[23:0]`:

| Word/Byte Offset | Field | Format |
|---|---|---|
| `0x00` | `MAGIC` | `0x54454E49` (`"INET"`) |
| `0x04` | `HEADER` | `{num_refs[15:0], version[15:0]}` |
| `0x08` | `boot_a` | Initial active pair port A (`24'b` zero-extended) |
| `0x0C` | `boot_b` | Initial active pair port B (`24'b` zero-extended) |
| `0x10 + 4*ref_id` | `dir_entry` | `{num_redex[3:0], num_wires[4:0], num_nodes[4:0], word_offset[17:0]}` |
| `word_offset * 4` | `template_body` | `root_port`, then `num_nodes × (aux0, aux1)`, then `num_redex × (ra, rb)` |

### 5.2 QSPI PSRAM A (`CS1_n`) & PSRAM B (`CS2_n`)

- **PSRAM A (`CS1_n`):**
  - `0x00_0000 – 0x03_FFFF`: Node Arena (`nid[14:0] << 3`, up to 32,768 nodes × 8B)
  - `0x04_0000 – 0x05_FFFF`: Wire Arena (`wid[14:0] << 2`, up to 32,768 wires × 4B)
- **PSRAM B (`CS2_n`):**
  - `0x00_0000+`: Active-pair spill ring (`pair_idx << 3`, 8B per `(port_a, port_b)` pair)

### 5.3 SWD Target (`inet_swd`) & Unified `DBG_ADDR[23:0]` Map

Standard ARM ADIv5 2-wire SWD (`SWCLK` on `ui[0]`, `SWDIO` on `uio[4]`), line reset (`>=50` ones) + JTAG-to-SWD (`0xE79E`), `DP IDCODE = 0x11E7_0001`, `AP IDR = 0x11E7_0010`:

- **DP Registers:** `0x0` `IDCODE`/`ABORT`, `0x4` `CTRL/STAT`, `0x8` `SELECT` (`APBANKSEL[7:4]`), `0xC` `RDBUFF`.
- **AP Bank 0 (Control & Memory):** `0x0` `DBG_CTRL` (`[0]=halt`, `[1]=step`, `[2]=soft_reset`, read returns status), `0x4` `DBG_ADDR[23:0]`, `0x8` `DBG_DATA` (posted read -> `RDBUFF`, write -> `dbg_mem_req`), `0xC` `IDR`.
- **AP Bank 1 (Telemetry):** `0x0` `cycle_cnt`, `0x4` `redex_cnt`, `0x8` `qspi_txn_cnt`, `0xC` `queue_depths`.
- **Unified `DBG_ADDR[23:20]` Regions (Core Halted):**
  - `0x0`: Core control/status, `ROOT[0..3]`, hot-hit/spill counters
  - `0x1`: Node Arena (`[14:0]` = `nid`, `[2]` = `aux0/aux1`)
  - `0x2`: Wire Arena (`[14:0]` = `wid`)
  - `0x3`: Tile Redex Queues
  - `0x4`: QSPI Flash (`CS0_n`) passthrough read
  - `0x5`: QSPI PSRAM A (`CS1_n`) passthrough R/W
  - `0x7`: Unified Peripheral MMIO passthrough (`dev = [11:8]`, `op = [5:2]`, `wdata = [7:0]` across `SERCOM`, `SYS`, `TIMER/PWM`, `PPI`, `EasyDMA`, `ACCEL`, `PIO`, `GPIO XBAR`)

---

## 6. Comparison vs ARM Cortex-M0+ & RISC-V (TinyQV / PicoRV32)

See `benchmarks/BENCHMARK_REPORT.md` for the full cycle, area, and QSPI-byte tables comparing `TT_INet_MCU` against ARM Cortex-M0+ and RISC-V (TinyQV, PicoRV32) across all 5 compiled workloads.

### Measured Full-Chip `tt_um_inet_mcu` RTL Results (`test/test.py`, Verilator)

| Benchmark | `ROOT(0)` | Reductions | Before (`8N/8W`) Cyc | **After (`16N/16W` Latch)** Cyc | Speedup / Reduction | QSPI Txns | Hot Hits | PSRAM B Spills |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `smoke_arith` | `80` (`0x300050`) | 17 | 2,270 | **1,442** | **-36.5%** | 12 | 14 | 0 |
| `periph_io` | `132` (`0x300084`) | 241 | 96,040 | **57,498** | **-40.1%** | 750 | 171 | 1 |
| `fsm_branch` | `46` (`0x30002e`) | 582 | 257,147 | **160,550** | **-37.6%** | 2,310 | 501 | 1 |
| `par_map_reduce` | `1464` (`0x3005b8`) | 729 | 295,088 | **237,691** | **-19.4%** | 3,520 | 264 | 10 |
| `fib_rec` | `21` (`0x300015`) | 871 | 348,129 | **198,094** | **-43.1%** | 2,901 | 769 | 6 |

- **Verification Signoff:** `pytest spec/formal_checks.py compiler/test_compiler.py` **13/13 PASS**; `test_peripherals` **4/4 PASS**; `test_io_bridge` **1/1 PASS**; `test_qspi_swd` **2/2 PASS**; `test/test.py` (`SIM=verilator`, full-chip SWD + Mode 1 Pmod + 5/5 Vine benchmarks bit-exact + `test_nordic_pio_pwm_xbar_dma`) **4/4 PASS**; `iverilog -g2012` compiles cleanly (`0` errors).

- **Architectural Trade-Off:**
  - **ARM Cortex-M0+ / TinyQV (RV32EC) / PicoRV32 (RV32IC)** excel on tight sequential scalar code (`fsm_branch`, raw arithmetic loops) where 16-bit Thumb/RVC instructions run in 1–4 cycles with high code density.
  - **TT_INet_MCU** eliminates register files (saving 512–1,024+ FFs), call stacks, RTOS context switches, and cache coherency hardware, spending that budget on 2-tile hardware multithreading, native fork/join parallelism (`par_map_reduce` reaches 48 concurrent active pairs), and non-blocking linear IO continuations that overlap Tile 1 reductions behind Tile 0 QSPI and UART/SPI/I2C latency.

---

## 7. Pre-Tapeout FPGA Prototyping Recommendation

**Recommendation: Yes — prototype on an inexpensive FPGA board before SKY130 tapeout.**

While our RTL, Yosys SKY130 synthesis, and cocotb behavioral models (W25Q128 + APS6404L + ADIv5 SWD + UART/SPI/I2C) validate logic and protocol framing, a real FPGA prototype de-risks three physical silicon issues that behavioral simulation cannot fully capture:
1. **Real QSPI Round-Trip Timing (`QSPI_LAT[1:0]`):** Verifies `0xEB` Quad Read capture margin against physical W25Q128 Flash and APS6404L PSRAM chips across clock speeds (12–50 MHz) and PCB trace lengths.
2. **Real SWD Debugger Interoperability:** Exercises `inet_swd` against an actual CMSIS-DAP / ST-Link / Raspberry Pi Pico Picoprobe + OpenOCD to verify line reset, `WAIT` turnaround retries, and live halt/step/dump.
3. **Live Peripheral & Long-Run Soak Tests:** Runs real USB-UART consoles, physical SPI/I2C sensors, and multi-million-reduction programs on `par_map_reduce` and `fib_rec` in seconds instead of minutes of RTL simulation.

### Recommended Low-Cost Boards Matched to `8x2` + QSPI Pmod

| Board | FPGA | Logic Capacity | Est. Price | Fit for `TT_INet_MCU` + QSPI Pmod |
|---|---|---|---:|---|
| **1BitSquared iCEBreaker** *(Best Plug-and-Play TT QSPI Pmod Match)* | Lattice iCE40UP5K | 5,280 LUT4 + 120 Kb BRAM + 1 Mb SPRAM | ~$70 | **3× standard Pmod headers** (direct plug-in for the `mole99` Tiny Tapeout Flash+PSRAM QSPI Pmod via `ui[6]=1`), fully open-source Yosys/nextpnr-ice40 flow, and official Tiny Tapeout FPGA demo compatibility. Ideal for 1–2 tile functional & timing validation. |
| **Sipeed Tang Nano 9K / 20K** *(Best Ultra-Low-Cost Option)* | Gowin GW1NR-9 / GW2AR-18 | 8,640 – 20,736 LUT4 | ~$20–$35 | Cheapest entry point with plenty of LUTs for 2–4 tile experiments + UART/USB-JTAG; requires a cheap 2×6 Pmod breakout header wired to the `mole99` QSPI Pmod. |
| **Lattice ECP5 (Colorlight i9 / ULX3S)** *(Best Full-Headroom + Logic Analyzer)* | Lattice ECP5-45F / 85F | 44k – 84k LUT4 | ~$40–$115 | Open-source Yosys/nextpnr-ecp5 flow with abundant LUTs/BRAM for full 2-tile (and scaled 4-tile) runs plus on-chip logic analyzer probing of QSPI/SWD transactions. |
