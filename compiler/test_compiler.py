"""End-to-end tests for .vine compiler & NetBuilder against INetMachine."""

from __future__ import annotations

from compiler.vine_compiler import compile_source
from spec.golden_model import INetMachine, MockPeripherals
from spec.isa import Tag, pack_port, unpack_flash_image


def _run_vine(src: str, schedule: str = "fifo", n_tiles: int = 2, seed: int = 0, periph=None):
    prog = compile_source(src)
    m = INetMachine(schedule=schedule, n_tiles=n_tiles, seed=seed, peripherals=periph)
    m.load_flash(prog.flash_bytes)
    res = m.run(max_steps=200000)
    assert res.live_nodes == 0 and res.live_wires == 0, res
    return prog, res


def test_arith_tuples_dup_era() -> None:
    src = """
    fn pair_sum(x, y) {
        let unused = x + 99;
        let t = (x + 1, y * 2);
        let (a, b) = t;
        let d = a + b + a;
        pack((d >>> 1) & 0xFF, (0x1234 >> 8) ^ 0x05)
    }
    fn main() {
        pair_sum(10, 7)
    }
    """
    prog, res = _run_vine(src)
    # a=11, b=14, d=36, (36>>>1)&FF=18=0x12, (0x12^0x05)=0x17 -> 0x1217
    assert res.roots[0] == pack_port(Tag.NUM, 0, 0x1217)
    img = unpack_flash_image(prog.flash_bytes)
    assert len(img.templates) == len(prog.fn_names)
    assert "pair_sum" in prog.sym_table["templates"]
    assert "REF(0)" in prog.disassembly


def test_recursive_fib_compiled_confluence() -> None:
    src = """
    fn fib(n) {
        switch n {
            0 => 0,
            m => switch m {
                0 => 1,
                k => fib(k) + fib(k + 1)
            }
        }
    }
    fn main() { fib(8) }
    """
    for sched, nt in [("fifo", 1), ("lifo", 1), ("rr_tiles", 2), ("rr_tiles", 4), ("random", 1)]:
        _, res = _run_vine(src, schedule=sched, n_tiles=nt, seed=42)
        assert res.roots[0] == pack_port(Tag.NUM, 0, 21)


def test_multiarg_freevar_capture_in_switch_and_if() -> None:
    src = """
    fn sum_to(n, acc) {
        switch n {
            0 => acc,
            p => sum_to(p, acc + p + 1)
        }
    }
    fn max2(a, b) {
        if a < b { b } else { a }
    }
    fn main() {
        let s = sum_to(10, 0);
        max2(s, 42)
    }
    """
    for sched in ("fifo", "lifo", "rr_tiles"):
        _, res = _run_vine(src, schedule=sched, n_tiles=4)
        assert res.roots[0] == pack_port(Tag.NUM, 0, 55)


def test_parallel_tree_map_reduce() -> None:
    src = """
    fn tree_sum(lo, len) {
        if len <= 1 {
            lo * 3 + 1
        } else {
            let half = len >> 1;
            tree_sum(lo, half) + tree_sum(lo + half, len - half)
        }
    }
    fn main() { tree_sum(0, 8) }
    """
    # sum_{i=0..7} (3*i + 1) = 3*28 + 8 = 92
    _, res = _run_vine(src, schedule="rr_tiles", n_tiles=4)
    assert res.roots[0] == pack_port(Tag.NUM, 0, 92)
    assert res.peak_active_pairs >= 4


def test_linear_peripherals_uart_spi_i2c() -> None:
    src = """
    fn main() {
        uart_tx('O');
        uart_tx('K');
        spi_cs_low();
        let rx = spi_xfer(0xA5);
        spi_cs_high();
        i2c_start();
        i2c_write(0xA0);
        i2c_write(0x5A);
        i2c_stop();
        rx & 0xFF
    }
    """
    for sched, nt in [("fifo", 1), ("lifo", 1), ("rr_tiles", 2), ("random", 1)]:
        periph = MockPeripherals(spi_rx_fn=lambda b: b ^ 0xFF)
        _, res = _run_vine(src, schedule=sched, n_tiles=nt, seed=11, periph=periph)
        assert res.uart_tx == [ord("O"), ord("K")]
        assert res.spi_tx == [0xA5]
        assert res.i2c_tx == [0xA0, 0x5A]
        assert res.roots[0] == pack_port(Tag.NUM, 0, 0x5A)
