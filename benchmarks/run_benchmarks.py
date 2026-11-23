"""Compile all benchmarks/*.vine, verify multi-schedule confluence, and emit JSON fixtures."""

from __future__ import annotations

import json
from pathlib import Path

from compiler.vine_compiler import compile_source
from spec.golden_model import INetMachine, MockPeripherals, make_benchmark_fixture
from spec.isa import Tag, unpack_port

ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "benchmarks" / "src"
FIX_DIR = ROOT / "benchmarks" / "fixtures"

BENCHMARK_ORDER = [
    "smoke_arith",
    "par_map_reduce",
    "fib_rec",
    "fsm_branch",
    "periph_io",
]


def _expected_python(name: str) -> int:
    if name == "smoke_arith":
        return ((17 + 25) * 2) - 4
    if name == "par_map_reduce":
        return sum(((i * 3) + 1) ^ 0x5A for i in range(16))
    if name == "fib_rec":
        a, b = 0, 1
        for _ in range(8):
            a, b = b, a + b
        return a
    if name == "fsm_branch":
        st, acc = 0, 1
        for n in range(12, 0, -1):
            p = n - 1
            s = st & 3
            if s == 0:
                st, acc = 1, (acc + p) ^ 0x13
            elif s == 1:
                st, acc = 2, ((acc << 1) + (p & 0xF)) & 0xFFFF
            elif s == 2:
                st, acc = 3, acc ^ ((p * 3) & 0xFFFF)
            else:
                st, acc = 0, ((acc >> 1) + p + 7) & 0xFFFF
        return acc
    if name == "periph_io":
        return 0x5A + sum(i * 5 + 3 for i in range(4))
    raise KeyError(name)


def run_all(write_fixtures: bool = True) -> list[dict]:
    FIX_DIR.mkdir(parents=True, exist_ok=True)
    summaries: list[dict] = []

    for name in BENCHMARK_ORDER:
        src_path = SRC_DIR / f"{name}.vine"
        prog = compile_source(src_path.read_text())
        expected_val = _expected_python(name)

        schedules = [
            ("fifo", 1, 0),
            ("lifo", 1, 0),
            ("rr_tiles", 2, 0),
            ("rr_tiles", 4, 0),
            *[("random", 1, s) for s in range(5)],
        ]

        ref_res = None
        for sched, nt, seed in schedules:
            periph = MockPeripherals(spi_rx_fn=lambda b: b ^ 0xFF)
            m = INetMachine(schedule=sched, n_tiles=nt, seed=seed, peripherals=periph)
            m.load_flash(prog.flash_bytes)
            res = m.run(max_steps=300000)
            tag, _, val = unpack_port(res.roots[0])
            assert tag == Tag.NUM and val == expected_val, (
                f"{name} ({sched},{nt},{seed}): got {(tag, val)}, expected {expected_val}"
            )
            assert res.live_nodes == 0 and res.live_wires == 0, f"{name}: leaked {res}"
            if ref_res is None:
                ref_res = res

        assert ref_res is not None
        fixture = make_benchmark_fixture(name, prog.flash_bytes, ref_res)
        fixture["expected_value"] = expected_val
        fixture["sym_table"] = prog.sym_table
        fixture["stats"] = {
            "flash_bytes": len(prog.flash_bytes),
            "num_templates": len(prog.fn_names),
            "reductions": ref_res.reductions,
            "rule_counts": ref_res.rule_counts,
            "peak_nodes": ref_res.peak_nodes,
            "peak_wires": ref_res.peak_wires,
            "peak_active_pairs": ref_res.peak_active_pairs,
            "flash_bytes_read": ref_res.flash_bytes_read,
            "psram_node_reads": ref_res.psram_node_reads,
            "psram_node_writes": ref_res.psram_node_writes,
            "psram_wire_ops": ref_res.psram_wire_ops,
        }

        if write_fixtures:
            (FIX_DIR / f"{name}.json").write_text(json.dumps(fixture, indent=2) + "\n")
        summaries.append(fixture)

    return summaries


if __name__ == "__main__":
    results = run_all(write_fixtures=True)
    for r in results:
        st = r["stats"]
        print(
            f"{r['name']:15s} | val={r['expected_value']:5d} | flash={st['flash_bytes']:4d}B "
            f"| redex={st['reductions']:5d} | peak_n={st['peak_nodes']:3d} "
            f"| peak_w={st['peak_wires']:3d} | peak_ap={st['peak_active_pairs']:2d} "
            f"| uart={r['expected_uart_tx']} spi={r['expected_spi_tx']} i2c={r['expected_i2c_tx']}"
        )
