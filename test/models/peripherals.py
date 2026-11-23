"""Behavioral UART, SPI Slave, and I2C Slave models for cocotb and unit tests."""

from __future__ import annotations

from collections import deque
from typing import Callable, Optional


class UartModel:
    """8N1 UART receiver (from DUT TX) and transmitter (to DUT RX) in clk cycles."""

    def __init__(self, clks_per_bit: int = 434):
        self.clks_per_bit = clks_per_bit
        self.rx_bytes: list[int] = []
        self._running = False

    async def monitor_tx(self, dut, tx_getter: Callable[[], int]) -> None:
        from cocotb.triggers import ClockCycles

        self._running = True
        while self._running:
            # Wait for start bit (line goes low)
            while self._running and tx_getter() != 0:
                await ClockCycles(dut.clk, 1)
            if not self._running:
                break
            # Sample center of bit 0 (1.5 bit times from start edge)
            await ClockCycles(dut.clk, self.clks_per_bit + self.clks_per_bit // 2)
            byte = 0
            for i in range(8):
                byte |= (tx_getter() & 1) << i
                await ClockCycles(dut.clk, self.clks_per_bit)
            # Stop bit should be 1; append captured byte
            self.rx_bytes.append(byte)

    async def send_byte(self, dut, rx_setter: Callable[[int], None], byte: int) -> None:
        from cocotb.triggers import ClockCycles

        rx_setter(0)  # Start bit
        await ClockCycles(dut.clk, self.clks_per_bit)
        for i in range(8):
            rx_setter((byte >> i) & 1)
            await ClockCycles(dut.clk, self.clks_per_bit)
        rx_setter(1)  # Stop bit
        await ClockCycles(dut.clk, self.clks_per_bit)

    def stop(self) -> None:
        self._running = False


class SpiSlaveModel:
    """SPI Mode 0 (CPOL=0, CPHA=0) peripheral slave model."""

    def __init__(self, tx_bytes: Optional[list[int]] = None):
        self.tx_queue: deque[int] = deque(tx_bytes or [])
        self.rx_bytes: list[int] = []
        self.cs_n = 1
        self.bit_cnt = 0
        self.rx_shift = 0
        self.tx_shift = 0
        self.miso = 0
        self._running = False

    def queue_tx(self, *bytes_out: int) -> None:
        for b in bytes_out:
            self.tx_queue.append(b & 0xFF)

    def on_cs_change(self, cs_n: int) -> int:
        cs_n = 1 if cs_n else 0
        if cs_n == self.cs_n:
            return self.miso
        self.cs_n = cs_n
        self.bit_cnt = 0
        self.rx_shift = 0
        if cs_n == 0:
            self.tx_shift = self.tx_queue.popleft() if self.tx_queue else 0xA5
            self.miso = (self.tx_shift >> 7) & 1
        else:
            self.miso = 0
        return self.miso

    def on_sck_rise(self, mosi: int) -> int:
        if self.cs_n:
            return self.miso
        self.rx_shift = ((self.rx_shift << 1) | (mosi & 1)) & 0xFF
        self.bit_cnt += 1
        if self.bit_cnt == 8:
            self.rx_bytes.append(self.rx_shift)
            self.bit_cnt = 0
            self.rx_shift = 0
            self.tx_shift = self.tx_queue.popleft() if self.tx_queue else 0xA5
        return self.miso

    def on_sck_fall(self) -> int:
        if self.cs_n:
            self.miso = 0
            return self.miso
        self.miso = (self.tx_shift >> (7 - self.bit_cnt)) & 1
        return self.miso

    async def run(
        self,
        dut,
        cs_getter: Callable[[], int],
        sck_getter: Callable[[], int],
        mosi_getter: Callable[[], int],
        miso_setter: Callable[[int], None],
    ) -> None:
        from cocotb.triggers import ReadOnly, RisingEdge, Timer

        self._running = True
        prev_sck = 0
        while self._running:
            await RisingEdge(dut.clk)
            await ReadOnly()
            cs_n = cs_getter()
            sck = sck_getter()
            mosi = mosi_getter()
            miso = self.on_cs_change(cs_n)
            if sck == 1 and prev_sck == 0:
                miso = self.on_sck_rise(mosi)
            elif sck == 0 and prev_sck == 1:
                miso = self.on_sck_fall()
            prev_sck = sck
            await Timer(1, unit="ps")
            miso_setter(miso)

    def stop(self) -> None:
        self._running = False


class I2cSlaveModel:
    """7-bit address open-drain I2C peripheral slave model."""

    def __init__(self, addr7: int = 0x50):
        self.addr7 = addr7 & 0x7F
        self.regs: dict[int, int] = {}
        self.txns: list[tuple[str, int, list[int]]] = []

        self.state = "IDLE"
        self.bit_cnt = 0
        self.shift = 0
        self.rw = 0
        self.reg_ptr = 0
        self.first_data = False
        self.sda_drive_low = False
        self._cur_data: list[int] = []
        self._prev_scl = 1
        self._prev_sda = 1
        self._running = False

    def step(self, scl: int, sda: int) -> int:
        """Advance I2C slave by one sample; returns open-drain sda_in level (0 or 1)."""
        scl = 1 if scl else 0
        sda = 1 if sda else 0

        # START / repeated START: SDA falling while SCL high
        if scl == 1 and self._prev_scl == 1 and self._prev_sda == 1 and sda == 0:
            if self.state != "IDLE" and self._cur_data:
                self.txns.append(("R" if self.rw else "W", self.addr7, list(self._cur_data)))
            self.state = "ADDR"
            self.bit_cnt = 0
            self.shift = 0
            self.sda_drive_low = False
            self._cur_data = []
        # STOP: SDA rising while SCL high
        elif scl == 1 and self._prev_scl == 1 and self._prev_sda == 0 and sda == 1:
            if self.state != "IDLE":
                self.txns.append(("R" if self.rw else "W", self.addr7, list(self._cur_data)))
            self.state = "IDLE"
            self.sda_drive_low = False
            self._cur_data = []
        # SCL rising edge: sample SDA
        elif scl == 1 and self._prev_scl == 0:
            if self.state == "ADDR":
                self.shift = ((self.shift << 1) | sda) & 0xFF
                self.bit_cnt += 1
                if self.bit_cnt == 8:
                    self.bit_cnt = 0
                    if (self.shift >> 1) == self.addr7:
                        self.rw = self.shift & 1
                        self.state = "ADDR_ACK"
                    else:
                        self.state = "IGNORE"
            elif self.state == "WRITE_BYTE":
                self.shift = ((self.shift << 1) | sda) & 0xFF
                self.bit_cnt += 1
                if self.bit_cnt == 8:
                    self.bit_cnt = 0
                    self._cur_data.append(self.shift)
                    if self.first_data:
                        self.reg_ptr = self.shift
                        self.first_data = False
                    else:
                        self.regs[self.reg_ptr] = self.shift
                        self.reg_ptr = (self.reg_ptr + 1) & 0xFF
                    self.state = "WRITE_ACK"
            elif self.state == "READ_MASTER_ACK":
                if sda == 0:
                    # Master ACKed -> send next register
                    self.reg_ptr = (self.reg_ptr + 1) & 0xFF
                    self.shift = self.regs.get(self.reg_ptr, 0x00)
                    self._cur_data.append(self.shift)
                    self.bit_cnt = 0
                    self.state = "READ_BYTE"
                else:
                    # Master NACKed -> wait for STOP/RESTART
                    self.state = "IGNORE"
        # SCL falling edge: update open-drain SDA output
        elif scl == 0 and self._prev_scl == 1:
            if self.state == "ADDR_ACK":
                self.sda_drive_low = True
                self.state = "ADDR_ACK_HOLD"
            elif self.state == "ADDR_ACK_HOLD":
                self.sda_drive_low = False
                if self.rw == 1:
                    self.shift = self.regs.get(self.reg_ptr, 0x00)
                    self._cur_data.append(self.shift)
                    self.bit_cnt = 0
                    bit = (self.shift >> 7) & 1
                    self.sda_drive_low = bit == 0
                    self.bit_cnt = 1
                    self.state = "READ_BYTE"
                else:
                    self.first_data = True
                    self.state = "WRITE_BYTE"
            elif self.state == "WRITE_ACK":
                self.sda_drive_low = True
                self.state = "WRITE_ACK_HOLD"
            elif self.state == "WRITE_ACK_HOLD":
                self.sda_drive_low = False
                self.state = "WRITE_BYTE"
            elif self.state == "READ_BYTE":
                if self.bit_cnt < 8:
                    bit = (self.shift >> (7 - self.bit_cnt)) & 1
                    self.sda_drive_low = bit == 0
                    self.bit_cnt += 1
                else:
                    self.sda_drive_low = False
                    self.state = "READ_MASTER_ACK"
            else:
                self.sda_drive_low = False

        self._prev_scl = scl
        self._prev_sda = sda
        return 0 if self.sda_drive_low else 1

    async def run(
        self,
        dut,
        scl_getter: Callable[[], int],
        sda_getter: Callable[[], int],
        sda_setter: Callable[[int], None],
    ) -> None:
        from cocotb.triggers import ReadOnly, RisingEdge, Timer

        self._running = True
        while self._running:
            await RisingEdge(dut.clk)
            await ReadOnly()
            sda_in = self.step(scl_getter(), sda_getter())
            await Timer(1, unit="ps")
            sda_setter(sda_in)

    def stop(self) -> None:
        self._running = False
