"""Single-source-of-truth ISA encoding & Flash XIP binary format for TT_INet_MCU."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum


class Tag(IntEnum):
    # TAG[3] == 0: atoms / substitution wires / sinks
    FREE = 0x0
    VAR = 0x1
    ERA = 0x2
    NUM = 0x3
    REF = 0x4
    ROOT = 0x5
    # TAG[3] == 1: 2-port heap nodes (aux0, aux1)
    CON = 0x8
    DUP = 0x9
    OPR = 0xA
    OP1 = 0xB
    SWI = 0xC
    IO = 0xD


def is_node_tag(tag: int) -> bool:
    """Return True if tag addresses a 2-port heap node."""
    return bool(int(tag) & 0x8)


class AluOp(IntEnum):
    ADD = 0
    SUB = 1
    AND = 2
    OR = 3
    XOR = 4
    SHL = 5
    SHR = 6
    SAR = 7
    EQ = 8
    NE = 9
    LTU = 10
    LTS = 11
    LEU = 12
    MUL = 13
    SEQ = 14
    PACK = 15


class SwiMode(IntEnum):
    DEC = 0  # if n == 0: choose aux0(ERA); else choose aux1(NUM(n-1))
    RAW = 1  # if n == 0: choose aux0(ERA); else choose aux1(NUM(n))


class IoDev(IntEnum):
    UART = 0
    SPI = 1
    I2C = 2
    SYS = 3
    TIMER = 4
    PPI = 5
    DMA = 6
    ACCEL = 7
    PIO = 8
    XBAR = 9


class UartOp(IntEnum):
    TX = 0
    RX_BLOCK = 1
    RX_POLL = 2
    STATUS = 3
    TX_WAIT = 4
    CFG_DIV_LO = 5
    CFG_DIV_HI = 6


class SpiOp(IntEnum):
    XFER = 0
    CS_LOW = 1
    CS_HIGH = 2
    STATUS = 3
    CFG_DIV = 4
    CFG_MODE = 5


class I2cOp(IntEnum):
    START = 0
    STOP = 1
    WRITE_BYTE = 2
    READ_ACK = 3
    READ_NACK = 4
    STATUS = 5
    CFG_DIV = 6


class SysOp(IntEnum):
    NOP = 0
    CYCLE_LO = 1
    CYCLE_HI = 2
    GPIO_RD = 3
    STATUS_LED = 4


def _s16(x: int) -> int:
    x &= 0xFFFF
    return x - 0x10000 if (x & 0x8000) else x


def eval_alu(op: int, a: int, b: int) -> int:
    """Evaluate exact 16-bit two's-complement ALU operation."""
    op = AluOp(op & 0xF)
    a &= 0xFFFF
    b &= 0xFFFF
    sh = b & 0xF
    if op == AluOp.ADD:
        return (a + b) & 0xFFFF
    if op == AluOp.SUB:
        return (a - b) & 0xFFFF
    if op == AluOp.AND:
        return a & b
    if op == AluOp.OR:
        return a | b
    if op == AluOp.XOR:
        return a ^ b
    if op == AluOp.SHL:
        return (a << sh) & 0xFFFF
    if op == AluOp.SHR:
        return a >> sh
    if op == AluOp.SAR:
        return (_s16(a) >> sh) & 0xFFFF
    if op == AluOp.EQ:
        return 1 if a == b else 0
    if op == AluOp.NE:
        return 1 if a != b else 0
    if op == AluOp.LTU:
        return 1 if a < b else 0
    if op == AluOp.LTS:
        return 1 if _s16(a) < _s16(b) else 0
    if op == AluOp.LEU:
        return 1 if a <= b else 0
    if op == AluOp.MUL:
        return (a * b) & 0xFFFF
    if op == AluOp.SEQ:
        return b
    if op == AluOp.PACK:
        return ((a & 0xFF) << 8) | (b & 0xFF)
    raise ValueError(f"Unknown AluOp: {op}")


def pack_port(tag: int, meta: int = 0, val: int = 0) -> int:
    """Pack a 24-bit tagged port: [23:20] TAG | [19:16] META | [15:0] VAL."""
    t = int(tag) & 0xF
    m = int(meta) & 0xF
    v = int(val) & 0xFFFF
    return (t << 20) | (m << 16) | v


def unpack_port(word: int) -> tuple[Tag, int, int]:
    """Unpack a 24-bit (or zero-extended 32-bit) port into (Tag, meta, val)."""
    w = int(word) & 0xFFFFFF
    return Tag((w >> 20) & 0xF), (w >> 16) & 0xF, w & 0xFFFF


def format_port(word: int) -> str:
    tag, meta, val = unpack_port(word)
    if tag == Tag.NUM:
        return f"NUM({val})"
    if tag == Tag.VAR:
        return f"VAR({val})"
    if tag == Tag.REF:
        return f"REF({val}, m={meta})"
    if tag == Tag.ROOT:
        return f"ROOT({val})"
    if tag == Tag.ERA:
        return "ERA"
    if tag == Tag.FREE:
        return "FREE"
    return f"{tag.name}(m={meta}, @{val})"


# 4B-aligned Flash XIP layout:
# 0x00: MAGIC ("INET" = 0x54454E49 LE)
# 0x04: {num_refs[15:0], version[15:0]}
# 0x08: boot_a (24-bit port in 32-bit LE word)
# 0x0C: boot_b (24-bit port in 32-bit LE word)
# 0x10 + 4*ref_id: dir_entry {num_redex[3:0], num_wires[4:0], num_nodes[4:0], word_offset[17:0]}
# At word_offset * 4:
#   word 0: root_port
#   words 1 .. 2*num_nodes: node[i].aux0, node[i].aux1
#   remaining 2*num_redex words: redex[j].a, redex[j].b

FLASH_MAGIC = 0x54454E49
FLASH_VERSION = 1


@dataclass(frozen=True)
class Template:
    name: str
    root_port: int
    nodes: list[tuple[int, int]]
    num_wires: int
    redexes: list[tuple[int, int]]


@dataclass(frozen=True)
class FlashImage:
    version: int
    boot_a: int
    boot_b: int
    templates: list[Template]
    dir_entries: list[int]
    raw_bytes: bytes


def pack_dir_entry(word_offset: int, num_nodes: int, num_wires: int, num_redex: int) -> int:
    if not (0 <= word_offset < (1 << 18)):
        raise ValueError(f"word_offset out of range: {word_offset}")
    if not (0 <= num_nodes < (1 << 5)):
        raise ValueError(f"num_nodes out of range: {num_nodes}")
    if not (0 <= num_wires < (1 << 5)):
        raise ValueError(f"num_wires out of range: {num_wires}")
    if not (0 <= num_redex < (1 << 4)):
        raise ValueError(f"num_redex out of range: {num_redex}")
    return (
        ((num_redex & 0xF) << 28)
        | ((num_wires & 0x1F) << 23)
        | ((num_nodes & 0x1F) << 18)
        | (word_offset & 0x3FFFF)
    )


def unpack_dir_entry(word: int) -> tuple[int, int, int, int]:
    """Return (word_offset, num_nodes, num_wires, num_redex)."""
    w = int(word) & 0xFFFFFFFF
    return (
        w & 0x3FFFF,
        (w >> 18) & 0x1F,
        (w >> 23) & 0x1F,
        (w >> 28) & 0xF,
    )


def pack_flash_image(
    templates: list[Template],
    boot_a: int,
    boot_b: int,
    version: int = FLASH_VERSION,
) -> bytes:
    num_refs = len(templates)
    if num_refs >= (1 << 16):
        raise ValueError("Too many templates")
    header_words = 4 + num_refs
    cur_word = header_words
    dir_words: list[int] = []
    body_words: list[int] = []

    for tmpl in templates:
        dir_words.append(
            pack_dir_entry(
                word_offset=cur_word,
                num_nodes=len(tmpl.nodes),
                num_wires=tmpl.num_wires,
                num_redex=len(tmpl.redexes),
            )
        )
        body_words.append(tmpl.root_port & 0xFFFFFF)
        for a0, a1 in tmpl.nodes:
            body_words.append(a0 & 0xFFFFFF)
            body_words.append(a1 & 0xFFFFFF)
        for ra, rb in tmpl.redexes:
            body_words.append(ra & 0xFFFFFF)
            body_words.append(rb & 0xFFFFFF)
        cur_word += 1 + 2 * len(tmpl.nodes) + 2 * len(tmpl.redexes)

    words = [
        FLASH_MAGIC,
        ((num_refs & 0xFFFF) << 16) | (version & 0xFFFF),
        boot_a & 0xFFFFFF,
        boot_b & 0xFFFFFF,
        *dir_words,
        *body_words,
    ]
    return b"".join(struct.pack("<I", w) for w in words)


def unpack_flash_image(data: bytes) -> FlashImage:
    if len(data) < 16 or (len(data) % 4) != 0:
        raise ValueError("Invalid flash image size")
    words = [struct.unpack_from("<I", data, i)[0] for i in range(0, len(data), 4)]
    if words[0] != FLASH_MAGIC:
        raise ValueError(f"Bad magic: 0x{words[0]:08x}")
    version = words[1] & 0xFFFF
    num_refs = (words[1] >> 16) & 0xFFFF
    boot_a = words[2] & 0xFFFFFF
    boot_b = words[3] & 0xFFFFFF
    if len(words) < 4 + num_refs:
        raise ValueError("Truncated directory")
    dir_entries = words[4 : 4 + num_refs]
    templates: list[Template] = []
    for idx, entry in enumerate(dir_entries):
        woff, n_nodes, n_wires, n_redex = unpack_dir_entry(entry)
        expected_len = 1 + 2 * n_nodes + 2 * n_redex
        if woff + expected_len > len(words):
            raise ValueError(f"Template {idx} out of bounds")
        pos = woff
        root_port = words[pos] & 0xFFFFFF
        pos += 1
        nodes = []
        for _ in range(n_nodes):
            nodes.append((words[pos] & 0xFFFFFF, words[pos + 1] & 0xFFFFFF))
            pos += 2
        redexes = []
        for _ in range(n_redex):
            redexes.append((words[pos] & 0xFFFFFF, words[pos + 1] & 0xFFFFFF))
            pos += 2
        templates.append(
            Template(
                name=f"ref_{idx}",
                root_port=root_port,
                nodes=nodes,
                num_wires=n_wires,
                redexes=redexes,
            )
        )
    return FlashImage(
        version=version,
        boot_a=boot_a,
        boot_b=boot_b,
        templates=templates,
        dir_entries=dir_entries,
        raw_bytes=data,
    )


def export_verilog_params() -> str:
    """Render ISA constants as Verilog localparams for RTL single-source sync."""
    lines = ["// Auto-generated from spec/isa.py"]
    for t in Tag:
        lines.append(f"localparam [3:0] TAG_{t.name:<4s} = 4'h{t.value:X};")
    for op in AluOp:
        lines.append(f"localparam [3:0] ALU_{op.name:<4s} = 4'h{op.value:X};")
    for m in SwiMode:
        lines.append(f"localparam [3:0] SWI_{m.name:<4s} = 4'h{m.value:X};")
    for d in IoDev:
        lines.append(f"localparam [3:0] IODEV_{d.name:<4s} = 4'h{d.value:X};")
    lines.append(f"localparam [31:0] INET_FLASH_MAGIC = 32'h{FLASH_MAGIC:08X};")
    return "\n".join(lines) + "\n"
