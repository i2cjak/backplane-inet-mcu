"""Vine-inspired Interaction-Net compiler (.vine -> QSPI Flash XIP + symbol table)."""

from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional

from compiler.net_builder import ProgramBuilder, SymPort, TemplateBuilder
from spec.isa import AluOp, I2cOp, IoDev, SpiOp, SwiMode, SysOp, UartOp


class CompileError(SyntaxError):
    """Raised on lex/parse/semantic errors in .vine source."""


# ---------------- Lexer ----------------

_TOKEN_SPEC = [
    ("WS", r"[ \t\r\n]+"),
    ("COMMENT", r"//[^\n]*"),
    ("HEX", r"0x[0-9a-fA-F]+"),
    ("NUM", r"[0-9]+"),
    ("CHAR", r"'(?:\\.|[^'\\])'"),
    ("OP2", r">>>|<<|>>|==|!=|<=|<s|=>"),
    ("IDENT", r"[A-Za-z_][A-Za-z0-9_]*"),
    ("SYM", r"[+\-*&|^~<>(){},;=]"),
]
_TOKEN_RE = re.compile("|".join(f"(?P<{k}>{v})" for k, v in _TOKEN_SPEC))

_KEYWORDS = {"fn", "let", "switch", "if", "else", "pack"}


@dataclass(frozen=True)
class Token:
    kind: str
    val: str
    line: int


def tokenize(src: str) -> list[Token]:
    tokens: list[Token] = []
    line = 1
    pos = 0
    while pos < len(src):
        m = _TOKEN_RE.match(src, pos)
        if not m:
            raise CompileError(f"Line {line}: unexpected char {src[pos]!r}")
        kind = m.lastgroup
        text = m.group()
        pos = m.end()
        if kind == "WS":
            line += text.count("\n")
            continue
        if kind == "COMMENT":
            continue
        if kind == "IDENT" and text in _KEYWORDS:
            kind = "KW"
        tokens.append(Token(kind, text, line))
    tokens.append(Token("EOF", "", line))
    return tokens


# ---------------- AST ----------------


@dataclass(frozen=True)
class NumExpr:
    value: int


@dataclass(frozen=True)
class VarExpr:
    name: str


@dataclass(frozen=True)
class TupleExpr:
    left: Expr
    right: Expr


@dataclass(frozen=True)
class UnaryExpr:
    op: str
    arg: Expr


@dataclass(frozen=True)
class BinExpr:
    op: AluOp
    left: Expr
    right: Expr


@dataclass(frozen=True)
class CallExpr:
    name: str
    args: list[Expr]


@dataclass(frozen=True)
class LetExpr:
    pattern: str | tuple[str, str]
    value: Expr
    body: Expr


@dataclass(frozen=True)
class SeqExpr:
    first: Expr
    second: Expr


@dataclass(frozen=True)
class SwitchExpr:
    scrut: Expr
    zero_branch: Expr
    nz_param: str
    nz_branch: Expr


@dataclass(frozen=True)
class IfExpr:
    cond: Expr
    then_branch: Expr
    else_branch: Expr


Expr = (
    NumExpr
    | VarExpr
    | TupleExpr
    | UnaryExpr
    | BinExpr
    | CallExpr
    | LetExpr
    | SeqExpr
    | SwitchExpr
    | IfExpr
)


@dataclass(frozen=True)
class FnDef:
    name: str
    params: list[str]
    body: Expr


# ---------------- Parser ----------------


class Parser:
    def __init__(self, tokens: list[Token]) -> None:
        self.toks = tokens
        self.i = 0

    def cur(self) -> Token:
        return self.toks[self.i]

    def eat(self, val: Optional[str] = None, kind: Optional[str] = None) -> Token:
        t = self.cur()
        if val is not None and t.val != val:
            raise CompileError(f"Line {t.line}: expected {val!r}, got {t.val!r}")
        if kind is not None and t.kind != kind:
            raise CompileError(f"Line {t.line}: expected {kind}, got {t.kind} ({t.val!r})")
        self.i += 1
        return t

    def match(self, val: str) -> bool:
        if self.cur().val == val:
            self.i += 1
            return True
        return False

    def parse_program(self) -> list[FnDef]:
        fns: list[FnDef] = []
        while self.cur().kind != "EOF":
            fns.append(self.parse_fn())
        return fns

    def parse_fn(self) -> FnDef:
        self.eat("fn", "KW")
        name = self.eat(kind="IDENT").val
        self.eat("(")
        params: list[str] = []
        if not self.match(")"):
            while True:
                params.append(self.eat(kind="IDENT").val)
                if self.match(")"):
                    break
                self.eat(",")
        self.eat("{")
        body = self.parse_block_exprs()
        self.eat("}")
        return FnDef(name=name, params=params, body=body)

    def parse_block_exprs(self) -> Expr:
        if self.cur().val == "let":
            self.eat("let", "KW")
            if self.match("("):
                a = self.eat(kind="IDENT").val
                self.eat(",")
                b = self.eat(kind="IDENT").val
                self.eat(")")
                pat: str | tuple[str, str] = (a, b)
            else:
                pat = self.eat(kind="IDENT").val
            self.eat("=")
            val = self.parse_expr()
            self.eat(";")
            body = self.parse_block_exprs()
            return LetExpr(pat, val, body)

        first = self.parse_expr()
        if self.match(";"):
            if self.cur().val == "}":
                return first
            rest = self.parse_block_exprs()
            return SeqExpr(first, rest)
        return first

    def parse_expr(self) -> Expr:
        return self.parse_cmp()

    def _bin(self, next_fn, op_map: dict[str, AluOp]) -> Expr:
        left = next_fn()
        while self.cur().val in op_map:
            op = op_map[self.cur().val]
            self.i += 1
            right = next_fn()
            left = BinExpr(op, left, right)
        return left

    def parse_cmp(self) -> Expr:
        return self._bin(
            self.parse_bitor,
            {
                "==": AluOp.EQ,
                "!=": AluOp.NE,
                "<": AluOp.LTU,
                "<s": AluOp.LTS,
                "<=": AluOp.LEU,
            },
        )

    def parse_bitor(self) -> Expr:
        return self._bin(self.parse_bitxor, {"|": AluOp.OR})

    def parse_bitxor(self) -> Expr:
        return self._bin(self.parse_bitand, {"^": AluOp.XOR})

    def parse_bitand(self) -> Expr:
        return self._bin(self.parse_shift, {"&": AluOp.AND})

    def parse_shift(self) -> Expr:
        return self._bin(
            self.parse_add,
            {"<<": AluOp.SHL, ">>": AluOp.SHR, ">>>": AluOp.SAR},
        )

    def parse_add(self) -> Expr:
        return self._bin(self.parse_mul, {"+": AluOp.ADD, "-": AluOp.SUB})

    def parse_mul(self) -> Expr:
        return self._bin(self.parse_unary, {"*": AluOp.MUL})

    def parse_unary(self) -> Expr:
        if self.cur().val in ("-", "~"):
            op = self.cur().val
            self.i += 1
            return UnaryExpr(op, self.parse_unary())
        return self.parse_atom()

    def parse_atom(self) -> Expr:
        t = self.cur()
        if t.kind == "HEX":
            self.i += 1
            return NumExpr(int(t.val, 16) & 0xFFFF)
        if t.kind == "NUM":
            self.i += 1
            return NumExpr(int(t.val, 10) & 0xFFFF)
        if t.kind == "CHAR":
            self.i += 1
            ch = bytes(t.val[1:-1], "utf-8").decode("unicode_escape")
            return NumExpr(ord(ch) & 0xFFFF)
        if t.val == "switch":
            self.eat("switch", "KW")
            scrut = self.parse_expr()
            self.eat("{")
            self.eat("0")
            self.eat("=>")
            z_expr = self.parse_block_exprs() if self.cur().val != "{" else self._parse_braced()
            self.eat(",")
            nz_param = self.eat(kind="IDENT").val
            self.eat("=>")
            nz_expr = self.parse_block_exprs() if self.cur().val != "{" else self._parse_braced()
            self.match(",")
            self.eat("}")
            return SwitchExpr(scrut, z_expr, nz_param, nz_expr)
        if t.val == "if":
            self.eat("if", "KW")
            cond = self.parse_expr()
            then_e = self._parse_braced()
            self.eat("else", "KW")
            else_e = self._parse_braced()
            return IfExpr(cond, then_e, else_e)
        if t.val == "pack":
            self.eat("pack", "KW")
            self.eat("(")
            a = self.parse_expr()
            self.eat(",")
            b = self.parse_expr()
            self.eat(")")
            return BinExpr(AluOp.PACK, a, b)
        if t.val == "(":
            self.eat("(")
            first = self.parse_expr()
            if self.match(","):
                second = self.parse_expr()
                self.eat(")")
                return TupleExpr(first, second)
            self.eat(")")
            return first
        if t.kind == "IDENT":
            self.i += 1
            if self.match("("):
                args: list[Expr] = []
                if not self.match(")"):
                    while True:
                        args.append(self.parse_expr())
                        if self.match(")"):
                            break
                        self.eat(",")
                return CallExpr(t.val, args)
            return VarExpr(t.val)
        raise CompileError(f"Line {t.line}: unexpected token {t.kind} {t.val!r}")

    def _parse_braced(self) -> Expr:
        self.eat("{")
        e = self.parse_block_exprs()
        self.eat("}")
        return e


# ---------------- Intrinsics & Analysis ----------------


def _io_intrinsic(name: str, args: list[Expr]) -> Optional[tuple[IoDev, Expr]]:
    """Map peripheral intrinsic calls to (IoDev, packed_cmd_expr)."""
    if name == "io" and len(args) == 2:
        # io(dev_num, cmd_expr)
        if not isinstance(args[0], NumExpr):
            raise CompileError("io(dev, cmd) requires constant dev ID")
        return IoDev(args[0].value), args[1]

    def mk_cmd(op: int, data_expr: Expr) -> Expr:
        return BinExpr(AluOp.PACK, NumExpr(op), data_expr)

    fixed: dict[str, tuple[IoDev, int, bool]] = {
        "uart_tx": (IoDev.UART, UartOp.TX, True),
        "uart_tx_wait": (IoDev.UART, UartOp.TX_WAIT, True),
        "uart_rx": (IoDev.UART, UartOp.RX_BLOCK, False),
        "uart_poll": (IoDev.UART, UartOp.RX_POLL, False),
        "uart_status": (IoDev.UART, UartOp.STATUS, False),
        "spi_xfer": (IoDev.SPI, SpiOp.XFER, True),
        "spi_cs_low": (IoDev.SPI, SpiOp.CS_LOW, False),
        "spi_cs_high": (IoDev.SPI, SpiOp.CS_HIGH, False),
        "spi_status": (IoDev.SPI, SpiOp.STATUS, False),
        "i2c_start": (IoDev.I2C, I2cOp.START, False),
        "i2c_stop": (IoDev.I2C, I2cOp.STOP, False),
        "i2c_write": (IoDev.I2C, I2cOp.WRITE_BYTE, True),
        "i2c_read_ack": (IoDev.I2C, I2cOp.READ_ACK, False),
        "i2c_read_nack": (IoDev.I2C, I2cOp.READ_NACK, False),
        "i2c_status": (IoDev.I2C, I2cOp.STATUS, False),
        "sys_cycle_lo": (IoDev.SYS, SysOp.CYCLE_LO, False),
        "sys_cycle_hi": (IoDev.SYS, SysOp.CYCLE_HI, False),
        "sys_led": (IoDev.SYS, SysOp.STATUS_LED, True),
    }
    if name not in fixed:
        return None
    dev, op, takes_arg = fixed[name]
    if takes_arg:
        if len(args) != 1:
            raise CompileError(f"{name} takes 1 argument")
        return dev, mk_cmd(op, args[0])
    if len(args) != 0:
        raise CompileError(f"{name} takes 0 arguments")
    return dev, NumExpr(op << 8)


def _free_vars(e: Expr, bound: set[str]) -> set[str]:
    if isinstance(e, NumExpr):
        return set()
    if isinstance(e, VarExpr):
        return set() if e.name in bound else {e.name}
    if isinstance(e, TupleExpr):
        return _free_vars(e.left, bound) | _free_vars(e.right, bound)
    if isinstance(e, UnaryExpr):
        return _free_vars(e.arg, bound)
    if isinstance(e, BinExpr):
        return _free_vars(e.left, bound) | _free_vars(e.right, bound)
    if isinstance(e, CallExpr):
        out: set[str] = set()
        for a in e.args:
            out |= _free_vars(a, bound)
        return out
    if isinstance(e, LetExpr):
        fv = _free_vars(e.value, bound)
        new_b = set(bound)
        if isinstance(e.pattern, tuple):
            new_b.update(e.pattern)
        else:
            new_b.add(e.pattern)
        return fv | _free_vars(e.body, new_b)
    if isinstance(e, SeqExpr):
        return _free_vars(e.first, bound) | _free_vars(e.second, bound)
    if isinstance(e, SwitchExpr):
        return (
            _free_vars(e.scrut, bound)
            | _free_vars(e.zero_branch, bound)
            | _free_vars(e.nz_branch, bound | {e.nz_param})
        )
    if isinstance(e, IfExpr):
        return (
            _free_vars(e.cond, bound)
            | _free_vars(e.then_branch, bound)
            | _free_vars(e.else_branch, bound)
        )
    return set()


def _has_io(e: Expr) -> bool:
    """Return True if expression performs peripheral IO or sequencing."""
    if isinstance(e, (NumExpr, VarExpr)):
        return False
    if isinstance(e, TupleExpr):
        return _has_io(e.left) or _has_io(e.right)
    if isinstance(e, UnaryExpr):
        return _has_io(e.arg)
    if isinstance(e, BinExpr):
        return _has_io(e.left) or _has_io(e.right)
    if isinstance(e, CallExpr):
        return _io_intrinsic(e.name, e.args) is not None or any(_has_io(a) for a in e.args)
    if isinstance(e, LetExpr):
        return _has_io(e.value) or _has_io(e.body)
    if isinstance(e, SeqExpr):
        return True
    if isinstance(e, SwitchExpr):
        return _has_io(e.scrut) or _has_io(e.zero_branch) or _has_io(e.nz_branch)
    if isinstance(e, IfExpr):
        return _has_io(e.cond) or _has_io(e.then_branch) or _has_io(e.else_branch)
    return False


def _count_template_uses(e: Expr, counts: dict[str, int]) -> None:
    """Count variable uses within the current template (lifted sub-templates count FV once)."""
    if isinstance(e, NumExpr):
        return
    if isinstance(e, VarExpr):
        if e.name in counts:
            counts[e.name] += 1
        return
    if isinstance(e, TupleExpr):
        _count_template_uses(e.left, counts)
        _count_template_uses(e.right, counts)
        return
    if isinstance(e, UnaryExpr):
        _count_template_uses(e.arg, counts)
        return
    if isinstance(e, BinExpr):
        _count_template_uses(e.left, counts)
        _count_template_uses(e.right, counts)
        return
    if isinstance(e, CallExpr):
        for a in e.args:
            _count_template_uses(a, counts)
        return
    if isinstance(e, LetExpr):
        _count_template_uses(e.value, counts)
        if isinstance(e.pattern, str) and _has_io(e.value):
            for fv in _free_vars(e.body, {e.pattern}):
                if fv in counts:
                    counts[fv] += 1
        else:
            _count_template_uses(e.body, counts)
        return
    if isinstance(e, SeqExpr):
        _count_template_uses(e.first, counts)
        for fv in _free_vars(e.second, set()):
            if fv in counts:
                counts[fv] += 1
        return
    if isinstance(e, SwitchExpr):
        _count_template_uses(e.scrut, counts)
        fvs = _free_vars(e.zero_branch, set()) | _free_vars(e.nz_branch, {e.nz_param})
        for fv in fvs:
            if fv in counts:
                counts[fv] += 1
        return
    if isinstance(e, IfExpr):
        _count_template_uses(e.cond, counts)
        fvs = _free_vars(e.then_branch, set()) | _free_vars(e.else_branch, set())
        for fv in fvs:
            if fv in counts:
                counts[fv] += 1
        return


# ---------------- Codegen ----------------


@dataclass(frozen=True)
class CompiledProgram:
    flash_bytes: bytes
    sym_table: dict
    disassembly: str
    fn_names: list[str]


class _Compiler:
    def __init__(self, fns: list[FnDef]) -> None:
        self.pb = ProgramBuilder()
        self.fns: dict[str, FnDef] = {}
        for fn in fns:
            if fn.name in self.fns:
                raise CompileError(f"Duplicate function {fn.name}")
            self.fns[fn.name] = fn
        if "main" not in self.fns:
            raise CompileError("Program must define fn main()")
        self.pending: list[FnDef] = list(fns)
        self._label_seq = 0
        self._gen_seq = 0

    def next_label(self) -> int:
        lbl = self._label_seq & 0xF
        self._label_seq += 1
        return lbl

    def next_helper(self, prefix: str) -> str:
        name = f"__{prefix}_{self._gen_seq}"
        self._gen_seq += 1
        return name

    def compile_all(self) -> CompiledProgram:
        while self.pending:
            fn = self.pending.pop(0)
            self._compile_fn(fn)
        flash_bytes, sym_table, disasm = self.pb.build_flash("main", 0)
        return CompiledProgram(
            flash_bytes=flash_bytes,
            sym_table=sym_table,
            disassembly=disasm,
            fn_names=list(sym_table["templates"].keys()),
        )

    def _make_fan(
        self, tb: TemplateBuilder, name: str, count: int, env: dict[str, list[SymPort]]
    ) -> SymPort:
        if count == 0:
            env[name] = []
            return tb.era()
        if count == 1:
            w = tb.wire(name)
            env[name] = [w]
            return w
        wires = [tb.wire(f"{name}_{i}") for i in range(count)]
        env[name] = list(wires)
        cur: list[SymPort] = list(wires)
        while len(cur) > 1:
            nxt: list[SymPort] = []
            i = 0
            while i < len(cur):
                if i + 1 < len(cur):
                    nxt.append(tb.dup(self.next_label(), cur[i], cur[i + 1]))
                    i += 2
                else:
                    nxt.append(cur[i])
                    i += 1
            cur = nxt
        return cur[0]

    def _compile_fn(self, fn: FnDef) -> None:
        tb = self.pb.new_template(fn.name)
        counts = {p: 0 for p in fn.params}
        _count_template_uses(fn.body, counts)
        env: dict[str, list[SymPort]] = {}

        param_ports = [self._make_fan(tb, p, counts[p], env) for p in fn.params]
        out_w = tb.wire(f"{fn.name}_ret")

        root: SymPort = out_w
        for pp in reversed(param_ports):
            root = tb.con(pp, root)
        tb.set_root(root)

        body_port = self._emit_expr(tb, fn.body, env)
        tb.redex(body_port, out_w)

    def _emit_expr(
        self, tb: TemplateBuilder, e: Expr, env: dict[str, list[SymPort]]
    ) -> SymPort:
        if isinstance(e, NumExpr):
            return tb.num(e.value)

        if isinstance(e, VarExpr):
            if e.name not in env or not env[e.name]:
                raise CompileError(f"Unbound or exhausted variable {e.name!r}")
            return env[e.name].pop(0)

        if isinstance(e, TupleExpr):
            lp = self._emit_expr(tb, e.left, env)
            rp = self._emit_expr(tb, e.right, env)
            return tb.con(lp, rp)

        if isinstance(e, UnaryExpr):
            if e.op == "-":
                return self._emit_expr(tb, BinExpr(AluOp.SUB, NumExpr(0), e.arg), env)
            if e.op == "~":
                return self._emit_expr(tb, BinExpr(AluOp.XOR, e.arg, NumExpr(0xFFFF)), env)
            raise CompileError(f"Unsupported unary op {e.op!r}")

        if isinstance(e, BinExpr):
            lp = self._emit_expr(tb, e.left, env)
            rp = self._emit_expr(tb, e.right, env)
            out_w = tb.wire("alu_out")
            tb.redex(lp, tb.opr(e.op, rp, out_w))
            return out_w

        if isinstance(e, CallExpr):
            io_hit = _io_intrinsic(e.name, e.args)
            if io_hit is not None:
                dev, cmd_expr = io_hit
                cmd_p = self._emit_expr(tb, cmd_expr, env)
                cont_w = tb.wire("io_cont")
                tb.redex(cmd_p, tb.io(dev, cont_w))
                return cont_w

            if e.name not in self.fns:
                raise CompileError(f"Undefined function {e.name!r}")
            target = self.fns[e.name]
            if len(e.args) != len(target.params):
                raise CompileError(
                    f"{e.name} expects {len(target.params)} args, got {len(e.args)}"
                )
            arg_ports = [self._emit_expr(tb, a, env) for a in e.args]
            out_w = tb.wire(f"{e.name}_out")
            chain: SymPort = out_w
            for ap in reversed(arg_ports):
                chain = tb.con(ap, chain)
            tb.redex(tb.ref(e.name), chain)
            return out_w

        if isinstance(e, LetExpr):
            if isinstance(e.pattern, tuple):
                a, b = e.pattern
                counts = {a: 0, b: 0}
                _count_template_uses(e.body, counts)
                sub_env = dict(env)
                pa = self._make_fan(tb, a, counts[a], sub_env)
                pb_port = self._make_fan(tb, b, counts[b], sub_env)
                val_p = self._emit_expr(tb, e.value, env)
                tb.redex(val_p, tb.con(pa, pb_port))
                return self._emit_expr(tb, e.body, sub_env)

            name = e.pattern
            if _has_io(e.value):
                # Enforce happens-before barrier so IO completes before body starts (even under LIFO).
                fvs = sorted(_free_vars(e.body, {name}) & set(env.keys()))
                z_name = self.next_helper("let_z")
                nz_name = self.next_helper("let_nz")
                z_fn = FnDef(z_name, fvs, LetExpr(name, NumExpr(0), e.body))
                nz_fn = FnDef(nz_name, [name, *fvs], e.body)
                self.fns[z_name] = z_fn
                self.fns[nz_name] = nz_fn
                self.pending.extend([z_fn, nz_fn])

                val_p = self._emit_expr(tb, e.value, env)
                fv_ports = [env[v].pop(0) for v in fvs]
                out_w = tb.wire("let_io_out")
                chain: SymPort = out_w
                for fp in reversed(fv_ports):
                    chain = tb.con(fp, chain)
                br = tb.con(tb.ref(z_name), tb.ref(nz_name))
                tb.redex(val_p, tb.swi(SwiMode.RAW, br, chain))
                return out_w

            counts = {name: 0}
            _count_template_uses(e.body, counts)
            sub_env = dict(env)
            target_p = self._make_fan(tb, name, counts[name], sub_env)
            val_p = self._emit_expr(tb, e.value, env)
            tb.redex(val_p, target_p)
            return self._emit_expr(tb, e.body, sub_env)

        if isinstance(e, SeqExpr):
            fvs = sorted(_free_vars(e.second, set()) & set(env.keys()))
            z_name = self.next_helper("seq_z")
            nz_name = self.next_helper("seq_nz")
            z_fn = FnDef(z_name, fvs, e.second)
            nz_body = CallExpr(z_name, [VarExpr(v) for v in fvs])
            nz_fn = FnDef(nz_name, ["__seq_unused", *fvs], nz_body)
            self.fns[z_name] = z_fn
            self.fns[nz_name] = nz_fn
            self.pending.extend([z_fn, nz_fn])

            first_p = self._emit_expr(tb, e.first, env)
            fv_ports = [env[v].pop(0) for v in fvs]
            out_w = tb.wire("seq_out")
            chain: SymPort = out_w
            for fp in reversed(fv_ports):
                chain = tb.con(fp, chain)
            br = tb.con(tb.ref(z_name), tb.ref(nz_name))
            tb.redex(first_p, tb.swi(SwiMode.RAW, br, chain))
            return out_w

        if isinstance(e, (SwitchExpr, IfExpr)):
            if isinstance(e, SwitchExpr):
                scrut = e.scrut
                z_expr = e.zero_branch
                nz_param = e.nz_param
                nz_expr = e.nz_branch
                mode = SwiMode.DEC
                bound_nz = {nz_param}
            else:
                scrut = e.cond
                z_expr = e.else_branch
                nz_param = "__if_unused"
                nz_expr = e.then_branch
                mode = SwiMode.RAW
                bound_nz = set()

            fvs = sorted((_free_vars(z_expr, set()) | _free_vars(nz_expr, bound_nz)) & set(env.keys()))
            z_name = self.next_helper("br_z")
            nz_name = self.next_helper("br_nz")
            z_fn = FnDef(z_name, fvs, z_expr)
            nz_fn = FnDef(nz_name, [nz_param, *fvs], nz_expr)
            self.fns[z_name] = z_fn
            self.fns[nz_name] = nz_fn
            self.pending.extend([z_fn, nz_fn])

            scrut_p = self._emit_expr(tb, scrut, env)
            fv_ports = [env[v].pop(0) for v in fvs]
            out_w = tb.wire("sw_out")
            chain: SymPort = out_w
            for fp in reversed(fv_ports):
                chain = tb.con(fp, chain)
            br = tb.con(tb.ref(z_name), tb.ref(nz_name))
            tb.redex(scrut_p, tb.swi(mode, br, chain))
            return out_w

        raise CompileError(f"Unsupported AST node: {type(e).__name__}")


def compile_source(src: str) -> CompiledProgram:
    """Compile a .vine program into a QSPI Flash XIP binary & symbol table."""
    fns = Parser(tokenize(src)).parse_program()
    return _Compiler(fns).compile_all()
