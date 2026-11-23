"""Executable golden operational semantics for TT_INet_MCU."""

from __future__ import annotations

import random
from collections import Counter, deque
from dataclasses import dataclass
from typing import Callable, Optional

from spec.isa import (
    AluOp,
    I2cOp,
    IoDev,
    SpiOp,
    SwiMode,
    SysOp,
    Tag,
    Template,
    UartOp,
    eval_alu,
    format_port,
    is_node_tag,
    pack_port,
    unpack_flash_image,
    unpack_port,
)


class LinearityError(RuntimeError):
    """Raised when attempting to duplicate or erase an affine/linear IO handle."""


class ReductionTypeError(RuntimeError):
    """Raised on an invalid active pair interaction."""


class MockPeripherals:
    """Deterministic peripheral model matching src/inet_{uart,spi,i2c}.v opcodes."""

    def __init__(
        self,
        uart_rx: Optional[list[int]] = None,
        spi_rx_fn: Optional[Callable[[int], int]] = None,
        i2c_regs: Optional[dict[int, int]] = None,
    ) -> None:
        self.uart_tx: list[int] = []
        self.uart_rx: deque[int] = deque(uart_rx or [])
        self.spi_tx: list[int] = []
        self.spi_rx_fn = spi_rx_fn
        self.spi_cs_low = False
        self.spi_rx_buf = 0
        self.i2c_tx: list[int] = []
        self.i2c_regs: dict[int, int] = dict(i2c_regs or {})
        self.i2c_rx_buf = 0
        self.i2c_bus_busy = False
        self.i2c_last_nack = False
        self._i2c_ptr = 0
        self._i2c_addr_pending = True
        self.sys_cycles = 0

    def exec(self, dev: int, op: int, data: int) -> int:
        d = IoDev(dev & 0xF)
        op &= 0xF
        data &= 0xFF
        status = 0
        rdata = 0

        if d == IoDev.UART:
            uop = UartOp(op)
            if uop == UartOp.TX:
                self.uart_tx.append(data)
                status = 0x02 | (0x01 if self.uart_rx else 0x00)
                rdata = data
            elif uop == UartOp.TX_WAIT:
                status = 0x01 if self.uart_rx else 0x00
            elif uop in (UartOp.RX_BLOCK, UartOp.RX_POLL):
                if self.uart_rx:
                    rdata = self.uart_rx.popleft() & 0xFF
                    status = 0x01
            elif uop == UartOp.STATUS:
                status = 0x01 if self.uart_rx else 0x00
        elif d == IoDev.SPI:
            sop = SpiOp(op)
            if sop == SpiOp.XFER:
                if self.spi_cs_low:
                    self.spi_tx.append(data)
                    rdata = (self.spi_rx_fn(data) if self.spi_rx_fn else (data ^ 0xFF)) & 0xFF
                else:
                    rdata = 0xFF
                self.spi_rx_buf = rdata
                status = 0x00 if self.spi_cs_low else 0x02
            elif sop == SpiOp.CS_LOW:
                self.spi_cs_low = True
                status = 0x00
                rdata = self.spi_rx_buf
            elif sop == SpiOp.CS_HIGH:
                self.spi_cs_low = False
                status = 0x02
                rdata = self.spi_rx_buf
            elif sop == SpiOp.STATUS:
                status = 0x00 if self.spi_cs_low else 0x02
                rdata = self.spi_rx_buf
        elif d == IoDev.I2C:
            iop = I2cOp(op)
            if iop == I2cOp.START:
                self.i2c_bus_busy = True
                self._i2c_addr_pending = True
                status = 0x04 | (0x02 if self.i2c_last_nack else 0x01)
                rdata = self.i2c_rx_buf
            elif iop == I2cOp.STOP:
                self.i2c_bus_busy = False
                self._i2c_addr_pending = True
                status = 0x02 if self.i2c_last_nack else 0x01
                rdata = self.i2c_rx_buf
            elif iop == I2cOp.WRITE_BYTE:
                self.i2c_tx.append(data)
                if self._i2c_addr_pending:
                    self._i2c_addr_pending = False
                else:
                    self._i2c_ptr = data
                self.i2c_last_nack = False
                status = (0x04 if self.i2c_bus_busy else 0x00) | 0x01
                rdata = self.i2c_rx_buf
            elif iop in (I2cOp.READ_ACK, I2cOp.READ_NACK):
                rdata = self.i2c_regs.get(self._i2c_ptr, 0xAA) & 0xFF
                self.i2c_rx_buf = rdata
                self._i2c_ptr = (self._i2c_ptr + 1) & 0xFF
                ack_bits = 0x02 if iop == I2cOp.READ_NACK else 0x01
                status = (0x04 if self.i2c_bus_busy else 0x00) | ack_bits
        elif d == IoDev.SYS:
            syop = SysOp(op)
            if syop == SysOp.CYCLE_LO:
                rdata = self.sys_cycles & 0xFF
            elif syop == SysOp.CYCLE_HI:
                rdata = (self.sys_cycles >> 8) & 0xFF
            elif syop == SysOp.STATUS_LED:
                rdata = data
        return ((status & 0xFF) << 8) | (rdata & 0xFF)


@dataclass
class RunResult:
    roots: dict[int, int]
    reductions: int
    rule_counts: dict[str, int]
    peak_nodes: int
    peak_wires: int
    peak_active_pairs: int
    live_nodes: int
    live_wires: int
    flash_bytes_read: int
    psram_node_reads: int
    psram_node_writes: int
    psram_wire_ops: int
    uart_tx: list[int]
    spi_tx: list[int]
    i2c_tx: list[int]


class INetMachine:
    """Small-step interaction-net machine with explicit 2-port nodes and 1-port wires."""

    def __init__(
        self,
        schedule: str = "fifo",
        n_tiles: int = 2,
        seed: int = 0,
        peripherals: Optional[MockPeripherals] = None,
    ) -> None:
        if schedule not in ("fifo", "lifo", "random", "rr_tiles"):
            raise ValueError(f"Unsupported schedule: {schedule}")
        self.schedule = schedule
        self.n_tiles = max(1, n_tiles)
        self.rng = random.Random(seed)
        self.peripherals = peripherals or MockPeripherals()

        self.nodes: dict[int, tuple[int, int]] = {}
        self.free_nodes: list[int] = []
        self.next_node_id = 0

        self.wires: dict[int, int] = {}
        self.free_wires: list[int] = []
        self.next_wire_id = 0

        self.roots: dict[int, int] = {}
        self.templates: list[Template] = []

        self._bag: deque[tuple[int, int]] = deque()
        self._tile_qs: list[deque[tuple[int, int]]] = [deque() for _ in range(self.n_tiles)]
        self._push_tile = 0
        self._pop_tile = 0

        self.reductions = 0
        self.rule_counts: Counter[str] = Counter()
        self.peak_nodes = 0
        self.peak_wires = 0
        self.peak_active_pairs = 0
        self.flash_bytes_read = 0
        self.psram_node_reads = 0
        self.psram_node_writes = 0
        self.psram_wire_ops = 0

    def alloc_node(self, aux0: int = 0, aux1: int = 0) -> int:
        nid = self.free_nodes.pop() if self.free_nodes else self._next_node()
        self.nodes[nid] = (aux0 & 0xFFFFFF, aux1 & 0xFFFFFF)
        self.psram_node_writes += 1
        if len(self.nodes) > self.peak_nodes:
            self.peak_nodes = len(self.nodes)
        return nid

    def _next_node(self) -> int:
        nid = self.next_node_id
        self.next_node_id += 1
        return nid

    def read_node(self, nid: int) -> tuple[int, int]:
        self.psram_node_reads += 1
        return self.nodes[nid]

    def write_node(self, nid: int, aux0: int, aux1: int) -> None:
        self.psram_node_writes += 1
        self.nodes[nid] = (aux0 & 0xFFFFFF, aux1 & 0xFFFFFF)

    def free_node(self, nid: int) -> None:
        del self.nodes[nid]
        self.free_nodes.append(nid)

    def alloc_wire(self) -> int:
        wid = self.free_wires.pop() if self.free_wires else self._next_wire()
        self.wires[wid] = pack_port(Tag.FREE)
        if len(self.wires) > self.peak_wires:
            self.peak_wires = len(self.wires)
        return wid

    def _next_wire(self) -> int:
        wid = self.next_wire_id
        self.next_wire_id += 1
        return wid

    def free_wire(self, wid: int) -> None:
        del self.wires[wid]
        self.free_wires.append(wid)

    def _active_count(self) -> int:
        if self.schedule == "rr_tiles":
            return sum(len(q) for q in self._tile_qs)
        return len(self._bag)

    def _push_pair(self, a: int, b: int) -> None:
        if self.schedule == "rr_tiles":
            self._tile_qs[self._push_tile].append((a, b))
            self._push_tile = (self._push_tile + 1) % self.n_tiles
        else:
            self._bag.append((a, b))
        cnt = self._active_count()
        if cnt > self.peak_active_pairs:
            self.peak_active_pairs = cnt

    def _pop_pair(self) -> Optional[tuple[int, int]]:
        if self.schedule == "rr_tiles":
            for _ in range(self.n_tiles):
                q = self._tile_qs[self._pop_tile]
                self._pop_tile = (self._pop_tile + 1) % self.n_tiles
                if q:
                    return q.popleft()
            return None
        if not self._bag:
            return None
        if self.schedule == "fifo":
            return self._bag.popleft()
        if self.schedule == "lifo":
            return self._bag.pop()
        idx = self.rng.randrange(len(self._bag))
        pair = self._bag[idx]
        del self._bag[idx]
        return pair

    def link(self, a: int, b: int) -> None:
        """Connect two ports, resolving atomic VAR substitutions & ROOT sinks."""
        while True:
            ta, _, va = unpack_port(a)
            tb, _, vb = unpack_port(b)
            if tb == Tag.VAR and ta != Tag.VAR:
                a, b = b, a
                ta, va = tb, vb
            if ta == Tag.VAR:
                self.psram_wire_ops += 1
                cur = self.wires[va]
                if unpack_port(cur)[0] == Tag.FREE:
                    self.wires[va] = b
                    return
                self.free_wire(va)
                a = cur
                continue
            break

        ta, _, va = unpack_port(a)
        tb, _, vb = unpack_port(b)
        if ta == Tag.ROOT and tb != Tag.REF:
            self.roots[va] = b
            return
        if tb == Tag.ROOT and ta != Tag.REF:
            self.roots[vb] = a
            return
        self._push_pair(a, b)

    def _expand_ref(self, ref_id: int, other: int) -> None:
        tmpl = self.templates[ref_id]
        self.flash_bytes_read += 4 + 4 * (1 + 2 * len(tmpl.nodes) + 2 * len(tmpl.redexes))
        nids = [self.alloc_node() for _ in tmpl.nodes]
        wids = [self.alloc_wire() for _ in range(tmpl.num_wires)]

        def reloc(port: int) -> int:
            t, m, v = unpack_port(port)
            if is_node_tag(t):
                return pack_port(t, m, nids[v])
            if t == Tag.VAR:
                return pack_port(t, m, wids[v])
            return port

        for idx, (a0, a1) in enumerate(tmpl.nodes):
            self.write_node(nids[idx], reloc(a0), reloc(a1))
        self.link(reloc(tmpl.root_port), other)
        for ra, rb in tmpl.redexes:
            self.link(reloc(ra), reloc(rb))

    def step(self) -> bool:
        pair = self._pop_pair()
        if pair is None:
            return False
        a, b = pair
        self.reductions += 1
        self.peripherals.sys_cycles += 1
        ta, ma, va = unpack_port(a)
        tb, mb, vb = unpack_port(b)

        if ta == Tag.ROOT and tb == Tag.REF:
            self._expand_ref(vb, a)
            self.rule_counts["REF"] += 1
            return True
        if tb == Tag.ROOT and ta == Tag.REF:
            self._expand_ref(va, b)
            self.rule_counts["REF"] += 1
            return True

        if ta == Tag.ERA or tb == Tag.ERA:
            other = b if ta == Tag.ERA else a
            to, _, vo = unpack_port(other)
            if to in (Tag.ERA, Tag.NUM, Tag.REF):
                self.rule_counts["ERA"] += 1
                return True
            if to == Tag.OP1:
                _, out = self.read_node(vo)
                self.free_node(vo)
                self.link(out, pack_port(Tag.ERA))
                self.rule_counts["ERA"] += 1
                return True
            if is_node_tag(to):
                if to == Tag.IO:
                    raise LinearityError("Cannot erase linear IO handle")
                p0, p1 = self.read_node(vo)
                self.free_node(vo)
                self.link(p0, pack_port(Tag.ERA))
                if unpack_port(p1)[0] != Tag.FREE:
                    self.link(p1, pack_port(Tag.ERA))
                self.rule_counts["ERA"] += 1
                return True
            raise ReductionTypeError(f"ERA cannot interact with {format_port(other)}")

        if (ta == Tag.REF and tb != Tag.DUP) or (tb == Tag.REF and ta != Tag.DUP):
            if ta == Tag.REF:
                self._expand_ref(va, b)
            else:
                self._expand_ref(vb, a)
            self.rule_counts["REF"] += 1
            return True

        if tb == Tag.DUP and ta != Tag.DUP:
            a, b = b, a
            ta, ma, va, tb, mb, vb = tb, mb, vb, ta, ma, va

        if ta == Tag.DUP:
            d0, d1 = self.read_node(va)
            self.free_node(va)
            if tb in (Tag.NUM, Tag.REF):
                self.link(d0, b)
                self.link(d1, b)
                self.rule_counts["DUP_ATOM"] += 1
                return True
            if tb == Tag.DUP and ma == mb:
                b0, b1 = self.read_node(vb)
                self.free_node(vb)
                self.link(d0, b0)
                self.link(d1, b1)
                self.rule_counts["DUP_ANNIH"] += 1
                return True
            if tb == Tag.OP1:
                num_a, out = self.read_node(vb)
                self.free_node(vb)
                op_a, op_b, nd = self.alloc_node(), self.alloc_node(), self.alloc_node()
                w0, w1 = self.alloc_wire(), self.alloc_wire()
                self.write_node(op_a, num_a, pack_port(Tag.VAR, 0, w0))
                self.write_node(op_b, num_a, pack_port(Tag.VAR, 0, w1))
                self.write_node(nd, pack_port(Tag.VAR, 0, w0), pack_port(Tag.VAR, 0, w1))
                self.link(d0, pack_port(Tag.OP1, mb, op_a))
                self.link(d1, pack_port(Tag.OP1, mb, op_b))
                self.link(out, pack_port(Tag.DUP, ma, nd))
                self.rule_counts["DUP_OP1"] += 1
                return True
            if tb == Tag.IO:
                raise LinearityError("Cannot duplicate linear IO handle")
            if tb in (Tag.CON, Tag.DUP, Tag.OPR, Tag.SWI):
                x0, x1 = self.read_node(vb)
                self.free_node(vb)
                nx0, nx1 = self.alloc_node(), self.alloc_node()
                nd0, nd1 = self.alloc_node(), self.alloc_node()
                w00, w01 = self.alloc_wire(), self.alloc_wire()
                w10, w11 = self.alloc_wire(), self.alloc_wire()
                self.write_node(nx0, pack_port(Tag.VAR, 0, w00), pack_port(Tag.VAR, 0, w10))
                self.write_node(nx1, pack_port(Tag.VAR, 0, w01), pack_port(Tag.VAR, 0, w11))
                self.write_node(nd0, pack_port(Tag.VAR, 0, w00), pack_port(Tag.VAR, 0, w01))
                self.write_node(nd1, pack_port(Tag.VAR, 0, w10), pack_port(Tag.VAR, 0, w11))
                self.link(d0, pack_port(tb, mb, nx0))
                self.link(d1, pack_port(tb, mb, nx1))
                self.link(x0, pack_port(Tag.DUP, ma, nd0))
                self.link(x1, pack_port(Tag.DUP, ma, nd1))
                self.rule_counts["DUP_COMM"] += 1
                return True
            raise ReductionTypeError(f"DUP cannot interact with {format_port(b)}")

        if ta == Tag.CON and tb == Tag.CON:
            a0, a1 = self.read_node(va)
            b0, b1 = self.read_node(vb)
            self.free_node(va)
            self.free_node(vb)
            self.link(a0, b0)
            self.link(a1, b1)
            self.rule_counts["CON_CON"] += 1
            return True

        if tb == Tag.NUM and ta != Tag.NUM:
            a, b = b, a
            ta, ma, va, tb, mb, vb = tb, mb, vb, ta, ma, va

        if ta == Tag.NUM and tb == Tag.OPR:
            rhs, out = self.read_node(vb)
            self.write_node(vb, a, out)
            self.link(rhs, pack_port(Tag.OP1, mb, vb))
            self.rule_counts["OPR"] += 1
            return True

        if ta == Tag.NUM and tb == Tag.OP1:
            lhs, out = self.read_node(vb)
            _, _, lhs_val = unpack_port(lhs)
            self.free_node(vb)
            res = eval_alu(mb, lhs_val, va)
            self.link(out, pack_port(Tag.NUM, 0, res))
            self.rule_counts["OP1"] += 1
            return True

        if ta == Tag.NUM and tb == Tag.SWI:
            br, out = self.read_node(vb)
            if va == 0:
                self.write_node(vb, out, pack_port(Tag.ERA))
                self.link(br, pack_port(Tag.CON, 0, vb))
            else:
                arg = ((va - 1) & 0xFFFF) if mb == SwiMode.DEC else va
                c2 = self.alloc_node(pack_port(Tag.NUM, 0, arg), out)
                self.write_node(vb, pack_port(Tag.ERA), pack_port(Tag.CON, 0, c2))
                self.link(br, pack_port(Tag.CON, 0, vb))
            self.rule_counts["SWI"] += 1
            return True

        if ta == Tag.NUM and tb == Tag.IO:
            cont, _ = self.read_node(vb)
            self.free_node(vb)
            op = (va >> 8) & 0xF
            data = va & 0xFF
            resp = self.peripherals.exec(mb, op, data)
            self.link(cont, pack_port(Tag.NUM, 0, resp))
            self.rule_counts["IO"] += 1
            return True

        raise ReductionTypeError(f"Unhandled active pair: {format_port(a)} >< {format_port(b)}")

    def load_flash(self, data: bytes) -> None:
        img = unpack_flash_image(data)
        self.templates = img.templates
        self.flash_bytes_read += 16
        self.link(img.boot_a, img.boot_b)

    def run(self, max_steps: int = 100000) -> RunResult:
        while self.step():
            if self.reductions >= max_steps:
                raise RuntimeError(f"Exceeded max_steps={max_steps}")
        return RunResult(
            roots=dict(self.roots),
            reductions=self.reductions,
            rule_counts=dict(self.rule_counts),
            peak_nodes=self.peak_nodes,
            peak_wires=self.peak_wires,
            peak_active_pairs=self.peak_active_pairs,
            live_nodes=len(self.nodes),
            live_wires=len(self.wires),
            flash_bytes_read=self.flash_bytes_read,
            psram_node_reads=self.psram_node_reads,
            psram_node_writes=self.psram_node_writes,
            psram_wire_ops=self.psram_wire_ops,
            uart_tx=list(self.peripherals.uart_tx),
            spi_tx=list(self.peripherals.spi_tx),
            i2c_tx=list(self.peripherals.i2c_tx),
        )


def make_benchmark_fixture(
    name: str,
    flash_bytes: bytes,
    result: RunResult,
    psram_init: Optional[dict[str, int]] = None,
) -> dict:
    """Emit per-benchmark JSON fixture matching @sim's cocotb harness schema."""
    return {
        "name": name,
        "flash_hex": flash_bytes.hex(),
        "psram_init": psram_init or {},
        "expected_root": result.roots.get(0, pack_port(Tag.FREE)),
        "expected_reductions": result.reductions,
        "expected_uart_tx": list(result.uart_tx),
        "expected_spi_tx": list(result.spi_tx),
        "expected_i2c_tx": list(result.i2c_tx),
    }
