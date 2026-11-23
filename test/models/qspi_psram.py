"""Behavioral APS6404L QSPI PSRAM model and shared QspiBus cocotb runner.

Supports SPI Mode 0 timing (sample IO on SCK rising edge, drive on SCK falling):
  - 0x66 / 0x99: Reset Enable / Reset
  - 0x9F: Read ID (0x0D, 0x5D, 0x50, 0xB1, 0x00, 0x00, 0x00, 0x00)
  - 0x35: Enter QPI mode (4-4-4)
  - 0xF5: Exit QPI mode (back to SPI)
  - 0x03: Read (1-1-1, 0 wait clocks)
  - 0x02: Write (1-1-1, 0 wait clocks)
  - 0xEB: Fast Read Quad (1-4-4 in SPI mode, 4-4-4 in QPI mode,
          configurable wait_cycles_eb [default 6 clocks])
  - 0x38: Quad Write (1-4-4 in SPI mode, 4-4-4 in QPI mode, 0 wait clocks)
"""

from __future__ import annotations

from typing import Callable, Optional


class APS6404LPsram:
    """Edge-accurate APS6404L 8MB QSPI PSRAM behavioral model."""

    DEVICE_ID = (0x0D, 0x5D, 0x50, 0xB1, 0x00, 0x00, 0x00, 0x00)

    def __init__(self, size_bytes: int = 8 * 1024 * 1024, wait_cycles_eb: int = 6):
        self.size_bytes = size_bytes
        self.wait_cycles_eb = wait_cycles_eb
        self.mem: dict[int, int] = {}
        self.qpi_mode = False
        self.reset_armed = False

        # Transaction state
        self.cs_n = 1
        self.state = "IDLE"
        self.cmd = 0
        self.bit_cnt = 0
        self.addr = 0
        self.wait_cnt = 0
        self.cur_byte = 0
        self.phase = 0
        self.id_idx = 0
        self.quad_data = False

        self.io_out = 0
        self.io_oe = 0

        # Telemetry
        self.read_txns = 0
        self.write_txns = 0
        self.bytes_read = 0
        self.bytes_written = 0
        self.sck_cycles = 0
        self.peak_addr = 0

    def _read_byte(self, addr: int) -> int:
        return self.mem.get(addr % self.size_bytes, 0x00)

    def _write_byte(self, addr: int, val: int) -> None:
        a = addr % self.size_bytes
        self.mem[a] = val & 0xFF
        if a > self.peak_addr:
            self.peak_addr = a

    def load_bytes(self, base_addr: int, data: bytes | bytearray | list[int]) -> None:
        for i, b in enumerate(data):
            self._write_byte(base_addr + i, b)

    def read_bytes(self, base_addr: int, length: int) -> bytes:
        return bytes(self._read_byte(base_addr + i) for i in range(length))

    def read_word32(self, addr: int) -> int:
        b = self.read_bytes(addr, 4)
        return int.from_bytes(b, "little")

    def write_word32(self, addr: int, val: int) -> None:
        self.load_bytes(addr, int(val & 0xFFFFFFFF).to_bytes(4, "little"))

    def reset_stats(self) -> None:
        self.read_txns = 0
        self.write_txns = 0
        self.bytes_read = 0
        self.bytes_written = 0
        self.sck_cycles = 0
        self.peak_addr = 0

    def stats(self) -> dict[str, int]:
        return {
            "read_txns": self.read_txns,
            "write_txns": self.write_txns,
            "bytes_read": self.bytes_read,
            "bytes_written": self.bytes_written,
            "sck_cycles": self.sck_cycles,
            "peak_addr": self.peak_addr,
        }

    def on_cs_change(self, cs_n: int) -> tuple[int, int]:
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
            self.phase = 0
            self.id_idx = 0
            self.io_out = 0
            self.io_oe = 0
        return self.io_out, self.io_oe

    def _enter_command(self, cmd: int) -> None:
        self.cmd = cmd
        self.bit_cnt = 0
        self.addr = 0
        self.wait_cnt = 0
        self.phase = 0
        if cmd == 0x66:
            self.reset_armed = True
            self.state = "DONE"
        elif cmd == 0x99:
            if self.reset_armed:
                self.qpi_mode = False
                self.reset_armed = False
            self.state = "DONE"
        else:
            self.reset_armed = False
            if cmd == 0x35 and not self.qpi_mode:
                self.qpi_mode = True
                self.state = "DONE"
            elif cmd == 0xF5 and self.qpi_mode:
                self.qpi_mode = False
                self.state = "DONE"
            elif cmd == 0x9F:
                self.state = "DEV_ID"
                self.id_idx = 0
                self.cur_byte = self.DEVICE_ID[0]
                self.quad_data = self.qpi_mode
            elif cmd in (0x03, 0x02, 0xEB, 0x38):
                self.state = "ADDR"
                if cmd in (0x03, 0xEB):
                    self.read_txns += 1
                else:
                    self.write_txns += 1
            else:
                self.state = "DONE"

    def on_sck_rise(self, io_in: int) -> tuple[int, int]:
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
            if self.cmd in (0xEB, 0x38) or self.qpi_mode:
                self.addr = ((self.addr << 4) | io_in) & 0xFFFFFF
                self.bit_cnt += 4
            else:
                self.addr = ((self.addr << 1) | (io_in & 1)) & 0xFFFFFF
                self.bit_cnt += 1

            if self.bit_cnt == 24:
                self.bit_cnt = 0
                self.phase = 0
                if self.cmd == 0x03:
                    self.quad_data = self.qpi_mode
                    self.cur_byte = self._read_byte(self.addr)
                    self.state = "DATA_OUT"
                elif self.cmd == 0xEB:
                    self.quad_data = True
                    self.wait_cnt = 0
                    self.state = "WAIT"
                elif self.cmd == 0x02:
                    self.quad_data = self.qpi_mode
                    self.cur_byte = 0
                    self.state = "DATA_IN"
                elif self.cmd == 0x38:
                    self.quad_data = True
                    self.cur_byte = 0
                    self.state = "DATA_IN"

        elif self.state == "WAIT":
            self.wait_cnt += 1
            if self.wait_cnt >= self.wait_cycles_eb:
                self.cur_byte = self._read_byte(self.addr)
                self.phase = 0
                self.state = "DATA_OUT"

        elif self.state == "DATA_IN":
            if self.quad_data:
                if self.phase == 0:
                    self.cur_byte = (io_in << 4) & 0xF0
                    self.phase = 1
                else:
                    self.cur_byte |= io_in & 0xF
                    self._write_byte(self.addr, self.cur_byte)
                    self.bytes_written += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = 0
                    self.phase = 0
            else:
                self.cur_byte = ((self.cur_byte << 1) | (io_in & 1)) & 0xFF
                self.phase += 1
                if self.phase == 8:
                    self._write_byte(self.addr, self.cur_byte)
                    self.bytes_written += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = 0
                    self.phase = 0

        return self.io_out, self.io_oe

    def on_sck_fall(self) -> tuple[int, int]:
        if self.cs_n:
            self.io_out = 0
            self.io_oe = 0
            return self.io_out, self.io_oe

        if self.state == "DATA_OUT":
            if self.quad_data:
                self.io_oe = 0xF
                if self.phase == 0:
                    self.io_out = (self.cur_byte >> 4) & 0xF
                    self.phase = 1
                else:
                    self.io_out = self.cur_byte & 0xF
                    self.bytes_read += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = self._read_byte(self.addr)
                    self.phase = 0
            else:
                bit = (self.cur_byte >> (7 - self.phase)) & 1
                self.io_oe = 0b0010
                self.io_out = bit << 1
                self.phase += 1
                if self.phase == 8:
                    self.bytes_read += 1
                    self.addr = (self.addr + 1) % self.size_bytes
                    self.cur_byte = self._read_byte(self.addr)
                    self.phase = 0

        elif self.state == "DEV_ID":
            if self.quad_data:
                self.io_oe = 0xF
                if self.phase == 0:
                    self.io_out = (self.cur_byte >> 4) & 0xF
                    self.phase = 1
                else:
                    self.io_out = self.cur_byte & 0xF
                    self.id_idx += 1
                    self.cur_byte = self.DEVICE_ID[self.id_idx % len(self.DEVICE_ID)]
                    self.phase = 0
            else:
                bit = (self.cur_byte >> (7 - self.phase)) & 1
                self.io_oe = 0b0010
                self.io_out = bit << 1
                self.phase += 1
                if self.phase == 8:
                    self.id_idx += 1
                    self.cur_byte = self.DEVICE_ID[self.id_idx % len(self.DEVICE_ID)]
                    self.phase = 0
        else:
            self.io_out = 0
            self.io_oe = 0

        return self.io_out, self.io_oe


class QspiBus:
    """Cocotb bus bridge connecting Flash (CS0_n) and PSRAMs (CS1_n, CS2_n) to TT ports.

    Supports both pinout modes proposed by @rtl:
      - mode="full" (ui[6]=0):
          CS0_n = uo_out[0], CS1_n = uo_out[1], SCK = uo_out[2],
          CS2_n = uio_out[7] (when uio_oe[7]==1, else pulled high),
          IO[3:0] = uio[3:0]
      - mode="pmod" (ui[6]=1, standard mole99 QSPI Pmod on uio[7:0]):
          CS0_n = uio[0], SD0 = uio[1], SD1 = uio[2], SCK = uio[3],
          SD2 = uio[4], SD3 = uio[5], CS1_n = uio[6], CS2_n = uio[7]
    """

    def __init__(
        self,
        dut,
        flash,
        psram_a: Optional[APS6404LPsram] = None,
        psram_b: Optional[APS6404LPsram] = None,
        mode: str = "full",
        uio_drive_cb: Optional[Callable[[int, int], None]] = None,
    ):
        self.dut = dut
        self.flash = flash
        self.psram_a = psram_a
        self.psram_b = psram_b
        self.mode = mode
        self.uio_drive_cb = uio_drive_cb
        self._running = False

    def _safe_int(self, val, default: int = 0) -> int:
        try:
            return int(val)
        except ValueError:
            return default

    def _read_pins(self) -> tuple[int, int, int, int, int]:
        uo = self._safe_int(self.dut.uo_out.value, 0xFF)
        uio_o = self._safe_int(self.dut.uio_out.value, 0xFF)
        uio_oe = self._safe_int(self.dut.uio_oe.value, 0x00)
        if self.mode == "pmod":
            cs0 = (uio_o >> 0) & 1 if (uio_oe & 0x01) else 1
            cs1 = (uio_o >> 6) & 1 if (uio_oe & 0x40) else 1
            cs2 = (uio_o >> 7) & 1 if (uio_oe & 0x80) else 1
            sck = (uio_o >> 3) & 1 if (uio_oe & 0x08) else 0
            io_in = (
                (((uio_o >> 1) & 1) << 0)
                | (((uio_o >> 2) & 1) << 1)
                | (((uio_o >> 4) & 1) << 2)
                | (((uio_o >> 5) & 1) << 3)
            )
            return cs0, cs1, cs2, sck, io_in
        cs0 = (uo >> 0) & 1
        cs1 = (uo >> 1) & 1
        cs2 = (uio_o >> 7) & 1 if (uio_oe & 0x80) else 1
        sck = (uo >> 2) & 1
        io_in = uio_o & 0xF
        return cs0, cs1, cs2, sck, io_in

    def _apply_io_to_uio(self, dev_out: int, dev_oe: int) -> None:
        if self.mode == "pmod":
            val = (
                (((dev_out >> 0) & 1) << 1)
                | (((dev_out >> 1) & 1) << 2)
                | (((dev_out >> 2) & 1) << 4)
                | (((dev_out >> 3) & 1) << 5)
            )
            mask = (
                (((dev_oe >> 0) & 1) << 1)
                | (((dev_oe >> 1) & 1) << 2)
                | (((dev_oe >> 2) & 1) << 4)
                | (((dev_oe >> 3) & 1) << 5)
            )
        else:
            val = dev_out & 0xF
            mask = dev_oe & 0xF

        if self.uio_drive_cb is not None:
            self.uio_drive_cb(mask, val)
            return
        cur = self._safe_int(self.dut.uio_in.value, 0)
        self.dut.uio_in.value = (cur & ~mask) | (val & mask)

    async def run(self) -> None:
        """Background cocotb task monitoring dut.clk edges to service QSPI transactions."""
        from cocotb.triggers import ReadOnly, RisingEdge, Timer

        self._running = True
        prev_sck = 0
        while self._running:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            cs0, cs1, cs2, sck, io_in = self._read_pins()

            devs = [(cs0, self.flash), (cs1, self.psram_a), (cs2, self.psram_b)]
            out_val = 0
            out_oe = 0

            for cs, dev in devs:
                if dev is None:
                    continue
                d_out, d_oe = dev.on_cs_change(cs)
                if sck == 1 and prev_sck == 0:
                    d_out, d_oe = dev.on_sck_rise(io_in)
                elif sck == 0 and prev_sck == 1:
                    d_out, d_oe = dev.on_sck_fall()
                if d_oe:
                    out_val |= d_out & d_oe
                    out_oe |= d_oe

            prev_sck = sck
            await Timer(1, unit="ps")
            self._apply_io_to_uio(out_val, out_oe)

    def stop(self) -> None:
        self._running = False
