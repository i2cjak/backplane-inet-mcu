"""Behavioral W25Q128 QSPI Flash model for cocotb and pure-Python unit tests.

Supports SPI Mode 0 sampling (sample IO on SCK rising edge, drive IO on SCK
falling edge):
  - 0x03: Read Data (1-1-1)
  - 0x0B: Fast Read (1-1-1, 8 dummy clocks)
  - 0x6B: Quad Output Fast Read (1-1-4, 8 dummy clocks)
  - 0xEB: Fast Read Quad I/O (1-4-4 in SPI mode, 4-4-4 in QPI mode;
          2 mode clocks + configurable dummy_clocks [default 4 = 6 total])
  - 0x9F: Read JEDEC ID (0xEF, 0x40, 0x18)
  - 0x38: Enter QPI mode (4-4-4)
  - 0xFF: Exit QPI mode (back to SPI)
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict


class W25Q128Flash:
    """Edge-accurate W25Q128 QSPI XIP flash behavioral model."""

    JEDEC_ID = (0xEF, 0x40, 0x18)

    def __init__(self, size_bytes: int = 16 * 1024 * 1024, dummy_clocks_eb: int = 4):
        self.size_bytes = size_bytes
        self.dummy_clocks_eb = dummy_clocks_eb  # +2 mode clocks = 6 total for 0xEB
        self.mem = bytearray([0xFF]) * min(size_bytes, 1024 * 1024)
        self._sparse: Dict[int, int] = {}
        self.qpi_mode = False

        # Transaction state
        self.cs_n = 1
        self.state = "IDLE"
        self.cmd = 0
        self.bit_cnt = 0
        self.addr = 0
        self.wait_cnt = 0
        self.wait_target = 0
        self.mode_byte = 0
        self.cur_byte = 0
        self.out_phase = 0
        self.id_idx = 0
        self.quad_out = False

        # Outputs (drive on SCK falling edge)
        self.io_out = 0
        self.io_oe = 0

        # Telemetry for benchmark comparison
        self.read_txns = 0
        self.bytes_read = 0
        self.sck_cycles = 0

    def _read_byte(self, addr: int) -> int:
        a = addr % self.size_bytes
        if a < len(self.mem):
            return self.mem[a]
        return self._sparse.get(a, 0xFF)

    def _write_backing(self, addr: int, val: int) -> None:
        a = addr % self.size_bytes
        if a < len(self.mem):
            self.mem[a] = val & 0xFF
        else:
            self._sparse[a] = val & 0xFF

    def load_bytes(self, base_addr: int, data: bytes | bytearray | list[int]) -> None:
        """Preload flash image bytes at base_addr."""
        for i, b in enumerate(data):
            self._write_backing(base_addr + i, b)

    def load_bin(self, path: str | Path, base_addr: int = 0) -> None:
        """Load raw binary file into flash at base_addr."""
        self.load_bytes(base_addr, Path(path).read_bytes())

    def load_hex(self, path: str | Path, base_addr: int = 0) -> None:
        """Load Verilog $readmemh-style hex file (one byte or 32-bit word per line)."""
        addr = base_addr
        for raw in Path(path).read_text(encoding="utf-8").splitlines():
            line = raw.split("//")[0].split("#")[0].strip()
            if not line:
                continue
            if line.startswith("@"):
                addr = int(line[1:], 16)
                continue
            for tok in line.split():
                val = int(tok, 16)
                if len(tok) <= 2:
                    self._write_backing(addr, val)
                    addr += 1
                elif len(tok) <= 4:
                    self._write_backing(addr, val & 0xFF)
                    self._write_backing(addr + 1, (val >> 8) & 0xFF)
                    addr += 2
                else:
                    for shift in (0, 8, 16, 24):
                        self._write_backing(addr, (val >> shift) & 0xFF)
                        addr += 1

    def reset_stats(self) -> None:
        self.read_txns = 0
        self.bytes_read = 0
        self.sck_cycles = 0

    def stats(self) -> dict[str, int]:
        return {
            "read_txns": self.read_txns,
            "bytes_read": self.bytes_read,
            "sck_cycles": self.sck_cycles,
        }

    def on_cs_change(self, cs_n: int) -> tuple[int, int]:
        """Update chip-select state; returns (io_out, io_oe)."""
        cs_n = 1 if cs_n else 0
        if cs_n == self.cs_n:
            return self.io_out, self.io_oe
        self.cs_n = cs_n
        if cs_n == 1:
            self.state = "IDLE"
            self.io_out = 0
            self.io_oe = 0
        else:
            self.state = "CMD"
            self.cmd = 0
            self.bit_cnt = 0
            self.addr = 0
            self.wait_cnt = 0
            self.mode_byte = 0
            self.out_phase = 0
            self.id_idx = 0
            self.io_out = 0
            self.io_oe = 0
        return self.io_out, self.io_oe

    def _enter_command(self, cmd: int) -> None:
        self.cmd = cmd
        self.bit_cnt = 0
        self.addr = 0
        self.wait_cnt = 0
        self.out_phase = 0
        if cmd == 0x9F:
            self.state = "JEDEC_ID"
            self.id_idx = 0
            self.cur_byte = self.JEDEC_ID[0]
            self.quad_out = self.qpi_mode
        elif cmd == 0x38 and not self.qpi_mode:
            self.qpi_mode = True
            self.state = "DONE"
        elif cmd == 0xFF and self.qpi_mode:
            self.qpi_mode = False
            self.state = "DONE"
        elif cmd in (0x03, 0x0B, 0x6B, 0xEB):
            self.state = "ADDR"
            self.read_txns += 1
        else:
            self.state = "DONE"

    def on_sck_rise(self, io_in: int) -> tuple[int, int]:
        """Sample inputs on SCK rising edge."""
        if self.cs_n:
            return self.io_out, self.io_oe
        self.sck_cycles += 1
        io_in &= 0xF

        if self.state == "CMD":
            if self.qpi_mode:
                self.cmd = ((self.cmd << 4) | io_in) & 0xFF
                self.bit_cnt += 4
            else:
                self.cmd = ((self.cmd << 1) | (io_in & 1)) & 0xFF
                self.bit_cnt += 1
            if self.bit_cnt == 8:
                self._enter_command(self.cmd)

        elif self.state == "ADDR":
            if self.cmd == 0xEB or self.qpi_mode:
                self.addr = ((self.addr << 4) | io_in) & 0xFFFFFF
                self.bit_cnt += 4
            else:
                self.addr = ((self.addr << 1) | (io_in & 1)) & 0xFFFFFF
                self.bit_cnt += 1

            if self.bit_cnt == 24:
                self.bit_cnt = 0
                if self.cmd == 0x03:
                    self.state = "DATA_OUT"
                    self.quad_out = self.qpi_mode
                    self.cur_byte = self._read_byte(self.addr)
                    self.out_phase = 0
                elif self.cmd in (0x0B, 0x6B):
                    self.state = "DUMMY"
                    self.wait_cnt = 0
                    self.wait_target = 8
                    self.quad_out = self.cmd == 0x6B or self.qpi_mode
                elif self.cmd == 0xEB:
                    self.state = "DUMMY"
                    self.wait_cnt = 0
                    self.wait_target = 2 + self.dummy_clocks_eb
                    self.quad_out = True

        elif self.state == "DUMMY":
            if self.cmd == 0xEB and self.wait_cnt < 2:
                self.mode_byte = ((self.mode_byte << 4) | io_in) & 0xFF
            self.wait_cnt += 1
            if self.wait_cnt >= self.wait_target:
                self.state = "DATA_OUT"
                self.cur_byte = self._read_byte(self.addr)
                self.out_phase = 0

        return self.io_out, self.io_oe

    def on_sck_fall(self) -> tuple[int, int]:
        """Drive outputs on SCK falling edge for the next rising edge."""
        if self.cs_n:
            self.io_out = 0
            self.io_oe = 0
            return self.io_out, self.io_oe

        if self.state == "DATA_OUT":
            if self.quad_out:
                self.io_oe = 0xF
                if self.out_phase == 0:
                    self.io_out = (self.cur_byte >> 4) & 0xF
                    self.out_phase = 1
                else:
                    self.io_out = self.cur_byte & 0xF
                    self.bytes_read += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = self._read_byte(self.addr)
                    self.out_phase = 0
            else:
                # 1-bit output on IO1 (MISO)
                bit = (self.cur_byte >> (7 - self.out_phase)) & 1
                self.io_oe = 0b0010
                self.io_out = bit << 1
                self.out_phase += 1
                if self.out_phase == 8:
                    self.bytes_read += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = self._read_byte(self.addr)
                    self.out_phase = 0

        elif self.state == "JEDEC_ID":
            if self.quad_out:
                self.io_oe = 0xF
                if self.out_phase == 0:
                    self.io_out = (self.cur_byte >> 4) & 0xF
                    self.out_phase = 1
                else:
                    self.io_out = self.cur_byte & 0xF
                    self.id_idx += 1
                    self.cur_byte = self.JEDEC_ID[self.id_idx % len(self.JEDEC_ID)]
                    self.out_phase = 0
            else:
                bit = (self.cur_byte >> (7 - self.out_phase)) & 1
                self.io_oe = 0b0010
                self.io_out = bit << 1
                self.out_phase += 1
                if self.out_phase == 8:
                    self.id_idx += 1
                    self.cur_byte = self.JEDEC_ID[self.id_idx % len(self.JEDEC_ID)]
                    self.out_phase = 0
        else:
            self.io_out = 0
            self.io_oe = 0

        return self.io_out, self.io_oe
