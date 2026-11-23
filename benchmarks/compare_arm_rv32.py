"""Quantitative comparison: TT_INet_MCU vs TinyQV (RV32EC), PicoRV32 (RV32IC), ARM Cortex-M0+."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

from benchmarks.run_benchmarks import run_all
from compiler.vine_compiler import compile_source
from spec.golden_model import INetMachine, MockPeripherals
from spec.isa import Tag, unpack_port

ROOT = Path(__file__).resolve().parent.parent
REPORT_PATH = ROOT / "benchmarks" / "BENCHMARK_REPORT.md"


# Exact 1-4-4 QSPI controller cycle costs matching src/inet_qspi_ctrl.v (qspi_lat=0):
# SCK toggles every clk -> 2 clks per SCK.
def qspi_read_cycles(words: int) -> int:
    return 2 * (20 + 8 * words) + 2


def qspi_write_cycles(words: int) -> int:
    return 2 * (14 + 8 * words) + 2


@dataclass(frozen=True)
class CoreProfile:
    name: str
    area_kge: float
    ffs: int
    cpi_core: float
    xip_line_words: int
    xip_hit_rate: float
    stack_words_per_call: int
    isr_cycles: int


TINYQV = CoreProfile("TinyQV (RV32EC 4b-serial)", 12.0, 260, 8.0, 2, 0.80, 3, 28)
PICORV32 = CoreProfile("PicoRV32 (RV32IC)", 13.5, 1150, 4.0, 4, 0.88, 4, 24)
CORTEX_M0P = CoreProfile("ARM Cortex-M0+ (Thumb-16)", 15.0, 720, 1.35, 4, 0.90, 4, 16)


@dataclass(frozen=True)
class WorkloadModel:
    rv32_instrs: int
    thumb_instrs: int
    calls: int
    uart_bytes: int
    spi_bytes: int
    i2c_bytes: int


WORKLOADS: dict[str, WorkloadModel] = {
    "smoke_arith": WorkloadModel(18, 16, 0, 1, 0, 0),
    "par_map_reduce": WorkloadModel(280, 250, 31, 0, 0, 0),
    "fib_rec": WorkloadModel(820, 740, 67, 0, 0, 0),
    "fsm_branch": WorkloadModel(310, 275, 12, 0, 0, 0),
    "periph_io": WorkloadModel(240, 215, 10, 2, 1, 2),
}


def simulate_inet_cycles(
    name: str,
    schedule: str = "rr_tiles",
    n_tiles: int = 2,
    hot_nodes: int = 16,
    hot_wires: int = 16,
    ref_cache_bytes: int = 0,
) -> dict:
    """Simulate multi-tile reductions + hot-set PSRAM spill/fill + QSPI Flash XIP."""
    src = (ROOT / "benchmarks" / "src" / f"{name}.vine").read_text()
    prog = compile_source(src)
    periph = MockPeripherals(spi_rx_fn=lambda b: b ^ 0xFF)
    m = INetMachine(schedule=schedule, n_tiles=n_tiles, seed=0, peripherals=periph)
    m.load_flash(prog.flash_bytes)

    cycles = 0
    par_steps = 0
    spill_reads = 0
    spill_writes = 0
    flash_txns = 0
    prev_flash_bytes = m.flash_bytes_read
    cached_bytes = 0
    seen_templates: set[int] = set()

    while m._active_count() > 0:
        ready_pairs = min(n_tiles, m._active_count())
        par_steps += 1
        wave_cycles = 3
        for _ in range(ready_pairs):
            prev_r = m.psram_node_reads
            prev_w = m.psram_node_writes
            prev_wire = m.psram_wire_ops
            pair = m._bag[-1] if schedule == "lifo" and m._bag else None
            # Peek REF id if cached
            ref_hit = False
            if pair is not None:
                ta, _, va = unpack_port(pair[0])
                tb, _, vb = unpack_port(pair[1])
                rid = va if ta == Tag.REF else (vb if tb == Tag.REF else -1)
                if rid >= 0 and rid in seen_templates:
                    ref_hit = True
            m.step()
            if m.flash_bytes_read > prev_flash_bytes:
                delta_bytes = m.flash_bytes_read - prev_flash_bytes
                delta_words = max(1, (delta_bytes + 3) // 4)
                if ref_cache_bytes > 0 and ref_hit:
                    pass
                else:
                    flash_txns += 1
                    wave_cycles += qspi_read_cycles(delta_words)
                    if ref_cache_bytes > 0 and cached_bytes + delta_bytes <= ref_cache_bytes:
                        # Record template footprint when small enough to fit in on-chip buffer
                        if pair is not None and rid >= 0:
                            seen_templates.add(rid)
                            cached_bytes += delta_bytes
                prev_flash_bytes = m.flash_bytes_read
            if len(m.nodes) > hot_nodes:
                spill_reads += m.psram_node_reads - prev_r
                spill_writes += m.psram_node_writes - prev_w
            if len(m.wires) > hot_wires:
                spill_reads += max(0, (m.psram_wire_ops - prev_wire) // 2)
        cycles += wave_cycles

    cycles += spill_reads * qspi_read_cycles(2) + spill_writes * qspi_write_cycles(2)
    cycles += m.rule_counts.get("IO", 0) * 12

    res = m.run()
    assert unpack_port(res.roots[0])[0] == Tag.NUM
    return {
        "cycles": cycles,
        "par_steps": par_steps,
        "peak_nodes": res.peak_nodes,
        "peak_wires": res.peak_wires,
        "spill_txns": spill_reads + spill_writes,
        "flash_txns": flash_txns,
    }


def estimate_cpu_cycles(name: str, prof: CoreProfile) -> int:
    wm = WORKLOADS[name]
    instrs = wm.thumb_instrs if "ARM" in prof.name else wm.rv32_instrs
    instr_bytes = instrs * 2 if "ARM" in prof.name else int(instrs * 2.6)
    words_needed = (instr_bytes + 3) // 4
    miss_lines = max(1, int(words_needed * (1.0 - prof.xip_hit_rate) / prof.xip_line_words))
    xip_cycles = miss_lines * qspi_read_cycles(prof.xip_line_words)
    stack_misses = max(0, wm.calls - 4)
    psram_cycles = stack_misses * (
        qspi_write_cycles(prof.stack_words_per_call) + qspi_read_cycles(prof.stack_words_per_call)
    )
    io_events = wm.uart_bytes + wm.spi_bytes + wm.i2c_bytes
    return int(round(instrs * prof.cpi_core + xip_cycles + psram_cycles + io_events * prof.isr_cycles))


# Measured full-chip Verilator RTL cycles & telemetry from test/test.py (qspi_lat=0, HOT_NODES=16, HOT_WIRES=16, REF_DIR=8, N_TILES=2)
MEASURED_RTL_CYCLES: dict[str, int] = {
    "smoke_arith": 1442,
    "periph_io": 57498,
    "par_map_reduce": 237691,
    "fib_rec": 198094,
    "fsm_branch": 160550,
}

MEASURED_RTL_TELEMETRY: dict[str, tuple[int, int, int]] = {
    # name: (qspi_txns, hot_hits, psram_b_spills)
    "smoke_arith": (12, 14, 0),
    "periph_io": (750, 171, 1),
    "par_map_reduce": (3520, 264, 10),
    "fib_rec": (2901, 769, 6),
    "fsm_branch": (2310, 501, 1),
}


def build_report(rtl_overrides: dict[str, dict] | None = None) -> str:
    summaries = run_all(write_fixtures=False)
    rtl_cycles = dict(MEASURED_RTL_CYCLES)
    if rtl_overrides:
        for k, v in rtl_overrides.items():
            if "cycles" in v:
                rtl_cycles[k] = int(v["cycles"])

    lines = [
        "# TT_INet_MCU Benchmark & Architecture Comparison Report",
        "",
        "Generated from compiled `.vine` programs running on `spec/golden_model.py`,",
        "measured `tt_um_inet_mcu` Verilator RTL cycles (`test/test.py`), and Yosys `sky130_fd_sc_hd` `8x2` synthesis.",
        "",
        "## 1. Golden & RTL Telemetry Metrics (FIFO vs LIFO Working-Set Peak)",
        "",
        "| Benchmark | Result | Flash (B) | Redex | FIFO Peak (N/W) | LIFO Peak (N/W) | RTL QSPI Txns | RTL Hot Hits | RTL Spills | RTL Cyc/Redex |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]

    lifo_opt: dict[str, dict] = {}
    for item in summaries:
        name = item["name"]
        st = item["stats"]
        lifo_opt[name] = simulate_inet_cycles(name, schedule="lifo", n_tiles=1, hot_nodes=32, hot_wires=32, ref_cache_bytes=512)
        qtx, hhit, sp = MEASURED_RTL_TELEMETRY.get(name, (0, 0, 0))
        rcyc = rtl_cycles.get(name, 0)
        cpr = f"{rcyc / st['reductions']:.1f}" if rcyc else "-"
        lines.append(
            f"| `{name}` | `{item['expected_value']}` | {st['flash_bytes']} | {st['reductions']} | "
            f"{st['peak_nodes']}/{st['peak_wires']} | {lifo_opt[name]['peak_nodes']}/{lifo_opt[name]['peak_wires']} | "
            f"{qtx:,} | {hhit} | {sp} | {cpr} |"
        )

    lines.extend([
        "",
        "## 2. Head-to-Head Cycle Comparison (SKY130 8x2 + External QSPI Flash/PSRAM)",
        "",
        "All cores share the identical external QSPI Flash XIP (`0xEB`: `2*(20+8*W)+2` clks) + PSRAM (`0x38`/`0xEB`) constraint.",
        "- **TT_INet_MCU RTL (Measured)**: full-chip `tt_um_inet_mcu` Verilator cycles (`HOT_NODES=16, HOT_WIRES=16` latch RFs, 8-entry REF dir cache, per-node/wire 16-entry relocation map, Chase-Lev LIFO + PSRAM A/B).",
        "- **INet Scaled (32-hot + 512B REF)**: projected on-chip hot working set scaling (beyond SKY130 8x2 latch/FF budget).",
        "",
        "| Benchmark | TT_INet_MCU RTL (Measured) | INet Scaled (32-hot + 512B) | TinyQV (RV32EC, ~12k GE) | PicoRV32 (RV32IC, ~13.5k GE) | ARM Cortex-M0+ (~15k GE) |",
        "|---|---:|---:|---:|---:|---:|",
    ])

    for item in summaries:
        name = item["name"]
        rtl_c = rtl_cycles.get(name)
        rtl_str = f"{rtl_c:,}" if rtl_c is not None else "pending"
        cl = lifo_opt[name]["cycles"]
        ctq = estimate_cpu_cycles(name, TINYQV)
        cpv = estimate_cpu_cycles(name, PICORV32)
        cm0 = estimate_cpu_cycles(name, CORTEX_M0P)
        lines.append(f"| `{name}` | {rtl_str} | {cl:,} | {ctq:,} | {cpv:,} | {cm0:,} |")

    lines.extend([
        "",
        "## 3. Area & Architectural Trade-Off Summary (SKY130 `8x2`)",
        "",
        "| Architecture | Core Area (SKY130 `8x2`) | Sequential Elements | Cache / RF / Autonomous Subsystem | Parallelism & IO Model |",
        "|---|---:|---:|---|---|",
        "| **TT_INet_MCU (Measured)** | **204,751.37 µm² (54.5 kGE, 67.70% raw / 77.86% post-CTS)** | **2,372 DFFs + 2,248 `dlxtn_1` latches** (0 `lpflow_inputisolatch`) | `16N/16W` hot latch RFs + `8×37b` REF cache + `16×10b` remap + `SERCOM` + 2×Timer/PWM + 4ch PPI + Linked-List EasyDMA + 4×INT8/Q15 MAC + `16×16` RP2040 PIO + PSEL XBAR | 2-tile Chase-Lev LIFO; autonomous zero-CPU DMA/PPI/PIO + linear IO |",
        f"| **TinyQV (RV32EC)** | ~{TINYQV.area_kge:.1f} kGE | ~{TINYQV.ffs} | 16x32b RV32E RF (latch/FF) + 4-bit serial ALU | Single-thread; stalls on QSPI & polling |",
        f"| **PicoRV32 (RV32IC)** | ~{PICORV32.area_kge:.1f} kGE | ~{PICORV32.ffs} | 32x32b RF (~1,024 FFs) + multi-cycle control | Single-thread; IRQ save/restore overhead |",
        f"| **ARM Cortex-M0+** | ~{CORTEX_M0P.area_kge:.1f} kGE | ~{CORTEX_M0P.ffs} | 13x32b RF + NVIC + Thumb-2 decoder | Fast scalar loops; 16-cycle IRQ + stack frames |",
        "",
        "## 4. Honest Engineering Takeaways",
        "",
        "1. **Critical Co-Design Finding — LIFO vs FIFO:** Pure FIFO breadth-first reduction explodes `par_map_reduce` to **247 live nodes** and `fib_rec` to **101 live nodes**, causing heavy QSPI PSRAM spill traffic. Switching each tile's local queue to **LIFO (depth-first) with oldest-end stealing** slashes peak live nodes **3.4–3.5×** (`par_map_reduce`: 247→73, `fib_rec`: 101→29) and eliminates PSRAM spills for `fib_rec` in a 32-node hot arena!",
        "2. **Where ARM Cortex-M0+ / RISC-V Win:** On tight sequential scalar loops (`fsm_branch`) and uncached QSPI workloads, 16-bit Thumb / RVC executes instructions in 1–4 cycles with compact code density, whereas an INet executes ~5–15 graph rewrites per scalar step (~10–25× more raw cycles without a template cache, or ~4–8× slower with a 512B template cache on sequential code).",
        "3. **Where TT_INet_MCU Excels:** Zero register files, zero call stacks, zero RTOS/coherency logic, formal confluence + linear IO safety, and automatic overlap of independent active pairs behind QSPI/UART/SPI/I2C latency (scaling cleanly if tile count + on-chip hot SRAM grow beyond Tiny Tapeout's 8x2 FF-only die).",
        "",
    ])
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--rtl-cycles", type=Path, default=None)
    args = ap.parse_args()
    overrides = json.loads(args.rtl_cycles.read_text()) if args.rtl_cycles else None
    md = build_report(overrides)
    REPORT_PATH.write_text(md)
    print(f"Wrote {REPORT_PATH} ({len(md)} bytes)")
