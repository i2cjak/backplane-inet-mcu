"""Symbolic Interaction-Net template builder, verifier & Flash XIP assembler."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from spec.isa import (
    AluOp,
    IoDev,
    SwiMode,
    Tag,
    Template,
    format_port,
    pack_flash_image,
    pack_port,
    unpack_dir_entry,
    unpack_flash_image,
)


class NetVerifyError(ValueError):
    """Raised when a template violates wire linearity or hardware directory limits."""


@dataclass(frozen=True)
class SymPort:
    kind: str  # "port" | "wire" | "ref"
    val: int = 0
    name: str = ""


class TemplateBuilder:
    """Builds a single closed interaction-net template with static linearity checks."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._nodes: list[tuple[SymPort, SymPort]] = []
        self._wires: list[str] = []
        self._redexes: list[tuple[SymPort, SymPort]] = []
        self._root: SymPort | None = None

    @staticmethod
    def era() -> SymPort:
        return SymPort("port", pack_port(Tag.ERA))

    @staticmethod
    def free() -> SymPort:
        return SymPort("port", pack_port(Tag.FREE))

    @staticmethod
    def num(value: int) -> SymPort:
        return SymPort("port", pack_port(Tag.NUM, 0, value & 0xFFFF))

    @staticmethod
    def root_sink(slot: int = 0) -> SymPort:
        return SymPort("port", pack_port(Tag.ROOT, 0, slot))

    @staticmethod
    def ref(name: str) -> SymPort:
        return SymPort("ref", 0, name)

    def wire(self, hint: str = "w") -> SymPort:
        wid = len(self._wires)
        self._wires.append(hint)
        return SymPort("wire", wid, hint)

    def _add_node(self, tag: Tag, meta: int, a0: SymPort, a1: SymPort) -> SymPort:
        nid = len(self._nodes)
        self._nodes.append((a0, a1))
        return SymPort("port", pack_port(tag, meta, nid))

    def con(self, aux0: SymPort, aux1: SymPort) -> SymPort:
        return self._add_node(Tag.CON, 0, aux0, aux1)

    def dup(self, label: int, aux0: SymPort, aux1: SymPort) -> SymPort:
        return self._add_node(Tag.DUP, label & 0xF, aux0, aux1)

    def opr(self, op: AluOp, rhs: SymPort, out: SymPort) -> SymPort:
        return self._add_node(Tag.OPR, int(op), rhs, out)

    def swi(self, mode: SwiMode, branches: SymPort, out: SymPort) -> SymPort:
        return self._add_node(Tag.SWI, int(mode), branches, out)

    def io(self, dev: IoDev, cont: SymPort) -> SymPort:
        return self._add_node(Tag.IO, int(dev), cont, self.free())

    def redex(self, a: SymPort, b: SymPort) -> None:
        self._redexes.append((a, b))

    def set_root(self, p: SymPort) -> None:
        self._root = p

    def build(self, resolve_ref: Callable[[str], int]) -> Template:
        if self._root is None:
            raise NetVerifyError(f"Template {self.name}: root_port not set")
        if len(self._nodes) > 31:
            raise NetVerifyError(f"Template {self.name}: {len(self._nodes)} nodes > 31 limit")
        if len(self._wires) > 31:
            raise NetVerifyError(f"Template {self.name}: {len(self._wires)} wires > 31 limit")
        if len(self._redexes) > 15:
            raise NetVerifyError(f"Template {self.name}: {len(self._redexes)} redexes > 15 limit")

        wire_uses = [0] * len(self._wires)

        def emit(p: SymPort) -> int:
            if p.kind == "wire":
                wire_uses[p.val] += 1
                return pack_port(Tag.VAR, 0, p.val)
            if p.kind == "ref":
                return pack_port(Tag.REF, 0, resolve_ref(p.name))
            return p.val

        root_word = emit(self._root)
        nodes = [(emit(a0), emit(a1)) for a0, a1 in self._nodes]
        redexes = [(emit(ra), emit(rb)) for ra, rb in self._redexes]

        for wid, cnt in enumerate(wire_uses):
            if cnt != 2:
                raise NetVerifyError(
                    f"Template {self.name}: wire {wid} ({self._wires[wid]}) used {cnt} times (must be 2)"
                )

        return Template(
            name=self.name,
            root_port=root_word,
            nodes=nodes,
            num_wires=len(self._wires),
            redexes=redexes,
        )


class ProgramBuilder:
    """Assembles multiple named TemplateBuilders into a Flash XIP image + symbol table."""

    def __init__(self) -> None:
        self._builders: dict[str, TemplateBuilder] = {}
        self._order: list[str] = []

    def new_template(self, name: str) -> TemplateBuilder:
        if name in self._builders:
            raise NetVerifyError(f"Duplicate template: {name}")
        tb = TemplateBuilder(name)
        self._builders[name] = tb
        self._order.append(name)
        return tb

    def ref_id(self, name: str) -> int:
        if name not in self._builders:
            raise NetVerifyError(f"Undefined template ref: {name}")
        return self._order.index(name)

    def build_flash(
        self,
        boot_template: str = "main",
        boot_slot: int = 0,
    ) -> tuple[bytes, dict, str]:
        templates = [self._builders[n].build(self.ref_id) for n in self._order]
        boot_a = pack_port(Tag.REF, 0, self.ref_id(boot_template))
        boot_b = pack_port(Tag.ROOT, 0, boot_slot)
        flash_bytes = pack_flash_image(templates, boot_a=boot_a, boot_b=boot_b)
        img = unpack_flash_image(flash_bytes)

        sym_table = {
            "magic": "INET",
            "version": img.version,
            "boot_a": boot_a,
            "boot_b": boot_b,
            "flash_bytes": len(flash_bytes),
            "templates": {},
        }
        lines = [
            f"; TT_INet_MCU Flash XIP ({len(flash_bytes)} bytes, {len(templates)} templates)",
            f"; boot: {format_port(boot_a)} >< {format_port(boot_b)}",
        ]
        for idx, (name, tmpl, dentry) in enumerate(zip(self._order, templates, img.dir_entries)):
            woff, nn, nw, nr = unpack_dir_entry(dentry)
            sym_table["templates"][name] = {
                "ref_id": idx,
                "word_offset": woff,
                "byte_addr": woff * 4,
                "num_nodes": nn,
                "num_wires": nw,
                "num_redex": nr,
            }
            lines.append(
                f"REF({idx}) {name} @ 0x{woff*4:06X} [nodes={nn} wires={nw} redex={nr}] root={format_port(tmpl.root_port)}"
            )
            for ni, (a0, a1) in enumerate(tmpl.nodes):
                lines.append(f"  node[{ni}]: ({format_port(a0)}, {format_port(a1)})")
            for ri, (ra, rb) in enumerate(tmpl.redexes):
                lines.append(f"  redex[{ri}]: {format_port(ra)} >< {format_port(rb)}")

        return flash_bytes, sym_table, "\n".join(lines) + "\n"
