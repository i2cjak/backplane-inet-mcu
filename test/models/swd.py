"""Behavioral ARM ADIv5 Serial Wire Debug (SWD) host model for cocotb and unit tests.

Implements exact ARM ADIv5 wire-level framing:
  - Line reset (>= 50 cycles SWDIO=1, >= 2 idle cycles SWDIO=0)
  - JTAG-to-SWD switch (0xE79E LSB-first)
  - 8-bit header (Start=1, APnDP, RnW, A[2], A[3], Parity, Stop=0, Park=1)
  - 1-cycle turnaround, 3-bit LSB-first ACK (OK=0b001, WAIT=0b010, FAULT=0b100)
  - 32-bit LSB-first data + even parity bit, automatic WAIT retry, idle trailing clocks
"""

from __future__ import annotations

from typing import Callable, Optional


class SwdError(RuntimeError):
    """Base error for SWD protocol failures."""


class SwdAckError(SwdError):
    """Raised on FAULT, WAIT exhaustion, or invalid ACK."""


class SwdParityError(SwdError):
    """Raised when read data parity does not match."""


def even_parity32(val: int) -> int:
    """Return even parity bit (0 or 1) of lower 32 bits of val."""
    return (val & 0xFFFFFFFF).bit_count() & 1


def swd_header(apndp: int, rnw: int, addr: int) -> list[int]:
    """Return the 8 SWD header bits in LSB-first wire transmission order."""
    ap = 1 if apndp else 0
    rw = 1 if rnw else 0
    a2 = (addr >> 2) & 1
    a3 = (addr >> 3) & 1
    parity = ap ^ rw ^ a2 ^ a3
    return [1, ap, rw, a2, a3, parity, 0, 1]


class SwdHost:
    """ARM ADIv5 SWD host driver usable with cocotb or a synchronous mock target."""

    ACK_OK = 0b001
    ACK_WAIT = 0b010
    ACK_FAULT = 0b100

    def __init__(
        self,
        dut=None,
        swclk_setter: Optional[Callable[[int], None]] = None,
        swdio_setter: Optional[Callable[[int], None]] = None,
        swdio_getter: Optional[Callable[[], int]] = None,
        clks_per_half_swclk: int = 4,
        max_wait_retries: int = 16,
        idle_cycles: int = 4,
    ):
        self.dut = dut
        self.swclk_setter = swclk_setter
        self.swdio_setter = swdio_setter
        self.swdio_getter = swdio_getter
        self.clks_per_half_swclk = max(1, clks_per_half_swclk)
        self.max_wait_retries = max_wait_retries
        self.idle_cycles = max(2, idle_cycles)

    async def _tick_bit(self, drive_bit: Optional[int]) -> int:
        """Drive (or release when None) SWDIO, pulse SWCLK high, and sample SWDIO."""
        from cocotb.triggers import ClockCycles, ReadOnly, Timer

        assert self.dut is not None
        assert self.swclk_setter is not None
        assert self.swdio_setter is not None
        assert self.swdio_getter is not None

        # Drive while SWCLK is low
        self.swclk_setter(0)
        if drive_bit is not None:
            self.swdio_setter(1 if drive_bit else 0)
        else:
            # Release line (pull-up = 1)
            self.swdio_setter(1)
        await ClockCycles(self.dut.clk, self.clks_per_half_swclk)

        # Rising edge of SWCLK
        self.swclk_setter(1)
        await ClockCycles(self.dut.clk, self.clks_per_half_swclk)
        await ReadOnly()
        sampled = self.swdio_getter() & 1

        # Return SWCLK low after leaving ReadOnly phase
        await Timer(1, unit="ps")
        self.swclk_setter(0)
        return sampled

    async def line_reset(self, reset_cycles: int = 52) -> None:
        """Send SWD line reset (>=50 high + idle low cycles)."""
        for _ in range(reset_cycles):
            await self._tick_bit(1)
        for _ in range(self.idle_cycles):
            await self._tick_bit(0)

    async def jtag_to_swd(self) -> None:
        """Send standard ARM JTAG-to-SWD switch sequence (0xE79E) + line reset."""
        for _ in range(52):
            await self._tick_bit(1)
        seq = 0xE79E
        for i in range(16):
            await self._tick_bit((seq >> i) & 1)
        await self.line_reset()

    async def _transfer_once(self, apndp: int, rnw: int, addr: int, wdata: int = 0) -> tuple[int, int]:
        """Execute a single SWD packet; returns (ack, rdata)."""
        for b in swd_header(apndp, rnw, addr):
            await self._tick_bit(b)

        # Turnaround (host -> target)
        await self._tick_bit(None)

        # 3-bit ACK, LSB-first
        ack = 0
        for i in range(3):
            bit = await self._tick_bit(None)
            ack |= bit << i

        if ack != self.ACK_OK:
            # Turnaround back to host + idle
            await self._tick_bit(None)
            for _ in range(self.idle_cycles):
                await self._tick_bit(0)
            return ack, 0

        if rnw:
            rdata = 0
            for i in range(32):
                bit = await self._tick_bit(None)
                rdata |= bit << i
            rpar = await self._tick_bit(None)
            # Turnaround back to host + trailing idle clocks
            await self._tick_bit(None)
            for _ in range(self.idle_cycles):
                await self._tick_bit(0)
            if rpar != even_parity32(rdata):
                raise SwdParityError(
                    f"SWD read parity error: rdata={rdata:#010x} got_par={rpar}"
                )
            return ack, rdata

        # Write path: turnaround back to host, then 32-bit WDATA + parity
        await self._tick_bit(None)
        for i in range(32):
            await self._tick_bit((wdata >> i) & 1)
        await self._tick_bit(even_parity32(wdata))
        for _ in range(self.idle_cycles):
            await self._tick_bit(0)
        return ack, 0

    async def transfer(self, apndp: int, rnw: int, addr: int, wdata: int = 0) -> int:
        """Execute SWD read/write with automatic WAIT retries."""
        for _ in range(self.max_wait_retries + 1):
            ack, rdata = await self._transfer_once(apndp, rnw, addr, wdata)
            if ack == self.ACK_OK:
                return rdata
            if ack == self.ACK_FAULT:
                raise SwdAckError(f"SWD FAULT on apndp={apndp} rnw={rnw} addr={addr:#x}")
            if ack != self.ACK_WAIT:
                raise SwdAckError(f"SWD invalid ACK={ack:#05b} on apndp={apndp} rnw={rnw} addr={addr:#x}")
        raise SwdAckError(f"SWD WAIT retries ({self.max_wait_retries}) exceeded")

    async def read_dp(self, addr: int) -> int:
        return await self.transfer(0, 1, addr)

    async def write_dp(self, addr: int, data: int) -> None:
        await self.transfer(0, 0, addr, data)

    async def read_ap(self, addr: int) -> int:
        return await self.transfer(1, 1, addr)

    async def write_ap(self, addr: int, data: int) -> None:
        await self.transfer(1, 0, addr, data)

    # Synchronous pure-Python bit-banged driver for unit self-testing without cocotb
    @staticmethod
    def sync_transfer(
        step_bit: Callable[[Optional[int]], int],
        apndp: int,
        rnw: int,
        addr: int,
        wdata: int = 0,
        max_wait_retries: int = 8,
        idle_cycles: int = 2,
    ) -> int:
        """Synchronous SWD transfer helper for standalone protocol unit tests."""
        for _ in range(max_wait_retries + 1):
            for b in swd_header(apndp, rnw, addr):
                step_bit(b)
            step_bit(None)  # Trn
            ack = 0
            for i in range(3):
                ack |= (step_bit(None) & 1) << i
            if ack == SwdHost.ACK_WAIT:
                step_bit(None)
                for _ in range(idle_cycles):
                    step_bit(0)
                continue
            if ack == SwdHost.ACK_FAULT:
                step_bit(None)
                for _ in range(idle_cycles):
                    step_bit(0)
                raise SwdAckError("SWD FAULT")
            if ack != SwdHost.ACK_OK:
                step_bit(None)
                for _ in range(idle_cycles):
                    step_bit(0)
                raise SwdAckError(f"SWD invalid ACK={ack:#05b}")

            if rnw:
                rdata = 0
                for i in range(32):
                    rdata |= (step_bit(None) & 1) << i
                rpar = step_bit(None) & 1
                step_bit(None)
                for _ in range(idle_cycles):
                    step_bit(0)
                if rpar != even_parity32(rdata):
                    raise SwdParityError("SWD read parity mismatch")
                return rdata

            step_bit(None)  # Trn to host
            for i in range(32):
                step_bit((wdata >> i) & 1)
            step_bit(even_parity32(wdata))
            for _ in range(idle_cycles):
                step_bit(0)
            return 0
        raise SwdAckError("SWD WAIT retries exceeded")
