"""Formal Z3 proofs & multi-schedule operational checks for TT_INet_MCU."""

from __future__ import annotations

import itertools
import z3

from spec.golden_model import INetMachine, LinearityError, make_benchmark_fixture
from spec.isa import (
    AluOp,
    I2cOp,
    IoDev,
    SpiOp,
    SwiMode,
    Tag,
    Template,
    UartOp,
    eval_alu,
    pack_dir_entry,
    pack_flash_image,
    pack_port,
    unpack_dir_entry,
    unpack_port,
)


def test_z3_port_encoding_bijective() -> None:
    s = z3.Solver()
    tag = z3.BitVec("tag", 4)
    meta = z3.BitVec("meta", 4)
    val = z3.BitVec("val", 16)
    packed = z3.Concat(tag, meta, val)
    s.add(
        z3.Or(
            z3.Extract(23, 20, packed) != tag,
            z3.Extract(19, 16, packed) != meta,
            z3.Extract(15, 0, packed) != val,
        )
    )
    assert s.check() == z3.unsat

    for t in Tag:
        is_node = bool(t.value & 0x8)
        assert (t in (Tag.CON, Tag.DUP, Tag.OPR, Tag.OP1, Tag.SWI, Tag.IO)) == is_node


def test_z3_flash_dir_bijective() -> None:
    s = z3.Solver()
    woff = z3.BitVec("woff", 18)
    nnodes = z3.BitVec("nnodes", 5)
    nwires = z3.BitVec("nwires", 5)
    nredex = z3.BitVec("nredex", 4)
    entry = z3.Concat(nredex, nwires, nnodes, woff)
    s.add(
        z3.Or(
            z3.Extract(17, 0, entry) != woff,
            z3.Extract(22, 18, entry) != nnodes,
            z3.Extract(27, 23, entry) != nwires,
            z3.Extract(31, 28, entry) != nredex,
        )
    )
    assert s.check() == z3.unsat
    assert unpack_dir_entry(pack_dir_entry(12345, 19, 27, 9)) == (12345, 19, 27, 9)


def test_z3_alu_properties() -> None:
    a = z3.BitVec("a", 16)
    b = z3.BitVec("b", 16)
    s = z3.Solver()
    s.add(
        z3.Or(
            (a + b) - b != a,
            (a ^ b) != (b ^ a),
            (a & b) != (b & a),
            (a | b) != (b | a),
            z3.And(a == b, a != b),
            z3.ULT(a, b) != z3.And(z3.ULE(a, b), a != b),
            z3.If(a < 0, (a >> 4) >= 0, (a >> 4) < 0),
        )
    )
    assert s.check() == z3.unsat

    assert eval_alu(AluOp.ADD, 0xFFFF, 1) == 0
    assert eval_alu(AluOp.SUB, 0, 1) == 0xFFFF
    assert eval_alu(AluOp.SAR, 0x8000, 4) == 0xF800
    assert eval_alu(AluOp.LTS, 0xFFFF, 0) == 1
    assert eval_alu(AluOp.LTU, 0xFFFF, 0) == 0
    assert eval_alu(AluOp.SEQ, 0x1234, 0x5678) == 0x5678
    assert eval_alu(AluOp.PACK, 0x00AB, 0x00CD) == 0xABCD


def test_rule_symmetry() -> None:
    def make_add(flip: bool) -> int:
        m = INetMachine()
        n = m.alloc_node(pack_port(Tag.NUM, 0, 17), pack_port(Tag.ROOT, 0, 0))
        pair = (
            (pack_port(Tag.OP1, AluOp.ADD, n), pack_port(Tag.NUM, 0, 25))
            if flip
            else (pack_port(Tag.NUM, 0, 25), pack_port(Tag.OP1, AluOp.ADD, n))
        )
        m.link(*pair)
        res = m.run()
        assert res.live_nodes == 0 and res.live_wires == 0
        return res.roots[0]

    assert make_add(False) == make_add(True) == pack_port(Tag.NUM, 0, 42)


def test_commutation_and_duplication_zero_leak() -> None:
    # DUP(0) >< CON(NUM(7), NUM(9)) consumed by two CON(ROOT, ROOT) sinks
    m = INetMachine()
    c_in = m.alloc_node(pack_port(Tag.NUM, 0, 7), pack_port(Tag.NUM, 0, 9))
    c_r0 = m.alloc_node(pack_port(Tag.ROOT, 0, 0), pack_port(Tag.ROOT, 0, 1))
    c_r1 = m.alloc_node(pack_port(Tag.ROOT, 0, 2), pack_port(Tag.ROOT, 0, 3))
    d_out = m.alloc_node(pack_port(Tag.CON, 0, c_r0), pack_port(Tag.CON, 0, c_r1))
    m.link(pack_port(Tag.CON, 0, c_in), pack_port(Tag.DUP, 0, d_out))
    res = m.run()
    assert res.live_nodes == 0 and res.live_wires == 0
    assert res.roots == {
        0: pack_port(Tag.NUM, 0, 7),
        1: pack_port(Tag.NUM, 0, 9),
        2: pack_port(Tag.NUM, 0, 7),
        3: pack_port(Tag.NUM, 0, 9),
    }

    # DUP(1) >< OP1(ADD, 10) with matching DUP(1) sink on OP1.out
    m2 = INetMachine()
    w0, w1 = m2.alloc_wire(), m2.alloc_wire()
    dup = m2.alloc_node(pack_port(Tag.VAR, 0, w0), pack_port(Tag.VAR, 0, w1))
    dup_sink = m2.alloc_node(pack_port(Tag.ROOT, 0, 0), pack_port(Tag.ROOT, 0, 1))
    op1 = m2.alloc_node(pack_port(Tag.NUM, 0, 10), pack_port(Tag.DUP, 1, dup_sink))
    m2.link(pack_port(Tag.DUP, 1, dup), pack_port(Tag.OP1, AluOp.ADD, op1))
    m2.link(pack_port(Tag.VAR, 0, w0), pack_port(Tag.NUM, 0, 3))
    m2.link(pack_port(Tag.VAR, 0, w1), pack_port(Tag.NUM, 0, 5))
    res2 = m2.run()
    assert res2.live_nodes == 0 and res2.live_wires == 0
    assert res2.roots == {
        0: pack_port(Tag.NUM, 0, 13),
        1: pack_port(Tag.NUM, 0, 15),
    }

    # Linearity check: DUP >< IO must raise LinearityError
    m3 = INetMachine()
    io_n = m3.alloc_node(pack_port(Tag.ROOT, 0, 0), pack_port(Tag.FREE))
    dup_n = m3.alloc_node(pack_port(Tag.ROOT, 0, 1), pack_port(Tag.ROOT, 0, 2))
    m3.link(pack_port(Tag.DUP, 0, dup_n), pack_port(Tag.IO, IoDev.UART, io_n))
    try:
        m3.run()
        raise AssertionError("Expected LinearityError")
    except LinearityError:
        pass


def test_var_var_chains_zero_leak() -> None:
    for perm in itertools.permutations([0, 1, 2]):
        m = INetMachine()
        ws = [m.alloc_wire(), m.alloc_wire(), m.alloc_wire()]
        edges = [
            (pack_port(Tag.NUM, 0, 99), pack_port(Tag.VAR, 0, ws[0])),
            (pack_port(Tag.VAR, 0, ws[0]), pack_port(Tag.VAR, 0, ws[1])),
            (pack_port(Tag.VAR, 0, ws[1]), pack_port(Tag.VAR, 0, ws[2])),
            (pack_port(Tag.VAR, 0, ws[2]), pack_port(Tag.ROOT, 0, 0)),
        ]
        for idx in perm:
            m.link(*edges[idx])
        m.link(*edges[3])
        res = m.run()
        assert res.roots[0] == pack_port(Tag.NUM, 0, 99)
        assert res.live_nodes == 0 and res.live_wires == 0


def _build_fib_flash(n: int) -> bytes:
    V = lambda i: pack_port(Tag.VAR, 0, i)
    N = lambda x: pack_port(Tag.NUM, 0, x)
    R = lambda rid: pack_port(Tag.REF, 0, rid)
    C = lambda nid: pack_port(Tag.CON, 0, nid)
    D = lambda lbl, nid: pack_port(Tag.DUP, lbl, nid)
    OP = lambda op, nid: pack_port(Tag.OPR, op, nid)
    SW = lambda mode, nid: pack_port(Tag.SWI, mode, nid)

    t_main = Template(
        name="main",
        root_port=V(0),
        nodes=[(N(n), V(0))],
        num_wires=1,
        redexes=[(R(1), C(0))],
    )
    t_fib = Template(
        name="fib",
        root_port=C(0),
        nodes=[
            (SW(SwiMode.DEC, 1), V(0)),
            (C(2), V(0)),
            (N(0), R(2)),
        ],
        num_wires=1,
        redexes=[],
    )
    t_fib_nz = Template(
        name="fib_nz",
        root_port=C(0),
        nodes=[
            (SW(SwiMode.DEC, 1), V(0)),
            (C(2), V(0)),
            (N(1), R(3)),
        ],
        num_wires=1,
        redexes=[],
    )
    t_fib_rec = Template(
        name="fib_rec",
        root_port=C(0),
        nodes=[
            (D(0, 1), V(0)),
            (OP(AluOp.ADD, 2), V(1)),
            (N(1), V(2)),
            (V(2), OP(AluOp.ADD, 5)),
            (V(1), V(3)),
            (V(3), V(0)),
        ],
        num_wires=4,
        redexes=[(R(1), C(3)), (R(1), C(4))],
    )
    return pack_flash_image(
        [t_main, t_fib, t_fib_nz, t_fib_rec],
        boot_a=R(0),
        boot_b=pack_port(Tag.ROOT, 0, 0),
    )


def test_recursive_fib_flash_roundtrip_and_confluence() -> None:
    flash = _build_fib_flash(8)
    schedules = [
        ("fifo", 1, 0),
        ("lifo", 1, 0),
        ("rr_tiles", 2, 0),
        ("rr_tiles", 4, 0),
        *[("random", 1, s) for s in range(20)],
    ]
    for sched, nt, seed in schedules:
        m = INetMachine(schedule=sched, n_tiles=nt, seed=seed)
        m.load_flash(flash)
        res = m.run()
        assert res.roots[0] == pack_port(Tag.NUM, 0, 21), (sched, nt, seed, res)
        assert res.live_nodes == 0 and res.live_wires == 0


def test_linear_io_ordering_and_fixture() -> None:
    V = lambda i: pack_port(Tag.VAR, 0, i)
    N = lambda x: pack_port(Tag.NUM, 0, x)
    IO = lambda dev, nid: pack_port(Tag.IO, dev, nid)
    OP = lambda op, nid: pack_port(Tag.OPR, op, nid)

    t_io = Template(
        name="io_seq",
        root_port=V(4),
        nodes=[
            (V(0), pack_port(Tag.FREE)),
            (N((UartOp.TX << 8) | ord("K")), IO(IoDev.UART, 2)),
            (V(1), pack_port(Tag.FREE)),
            (N(SpiOp.CS_LOW << 8), IO(IoDev.SPI, 4)),
            (V(2), pack_port(Tag.FREE)),
            (N((SpiOp.XFER << 8) | 0xA5), IO(IoDev.SPI, 6)),
            (V(3), pack_port(Tag.FREE)),
            (N((I2cOp.WRITE_BYTE << 8) | 0x5A), IO(IoDev.I2C, 8)),
            (V(4), pack_port(Tag.FREE)),
        ],
        num_wires=5,
        redexes=[
            (N((UartOp.TX << 8) | ord("O")), IO(IoDev.UART, 0)),
            (V(0), OP(AluOp.SEQ, 1)),
            (V(1), OP(AluOp.SEQ, 3)),
            (V(2), OP(AluOp.SEQ, 5)),
            (V(3), OP(AluOp.SEQ, 7)),
        ],
    )
    flash = pack_flash_image([t_io], boot_a=pack_port(Tag.REF, 0, 0), boot_b=pack_port(Tag.ROOT, 0, 0))
    res_ref = None
    for sched in ("fifo", "lifo", "random"):
        m = INetMachine(schedule=sched, seed=7)
        m.load_flash(flash)
        res = m.run()
        assert res.uart_tx == [ord("O"), ord("K")]
        assert res.spi_tx == [0xA5]
        assert res.i2c_tx == [0x5A]
        assert res.live_nodes == 0 and res.live_wires == 0
        res_ref = res

    fx = make_benchmark_fixture("io_seq", flash, res_ref)
    for k in (
        "name",
        "flash_hex",
        "psram_init",
        "expected_root",
        "expected_reductions",
        "expected_uart_tx",
        "expected_spi_tx",
        "expected_i2c_tx",
    ):
        assert k in fx
