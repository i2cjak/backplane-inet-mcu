"""Top-port-only cocotb verification & benchmark suite for tt_um_inet_mcu.

Compatible with both RTL and Tiny Tapeout gate-level (GATES=yes) simulation:
all checks interact exclusively through Tiny Tapeout top-level ports
(ui_in, uo_out, uio_in, uio_out, uio_oe, ena, clk, rst_n).
"""

from __future__ import annotations

import json
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

from models import (
    APS6404LPsram,
    I2cSlaveModel,
    SpiSlaveModel,
    SwdHost,
    UartModel,
    W25Q128Flash,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURES_DIR = REPO_ROOT / "benchmarks" / "fixtures"
BENCHMARK_ORDER = [
    "smoke_arith",
    "periph_io",
    "fib_rec",
    "fsm_branch",
    "par_map_reduce",
]


def _safe_int(val, default: int = 0) -> int:
    try:
        return int(val)
    except ValueError:
        return default


class BoardHarness:
    """Simulated Tiny Tapeout demoboard + QSPI Flash/PSRAM + SWD + UART/SPI/I2C."""

    def __init__(
        self,
        dut,
        flash_bytes: bytes = b"",
        psram_a_init: dict[int, int] | None = None,
        pmod_mode: bool = False,
        qspi_lat: int = 0,
        boot_halt: bool = False,
        spi_tx_bytes: list[int] | None = None,
        i2c_addr: int = 0x50,
    ):
        self.dut = dut
        self.pmod_mode = pmod_mode
        self.qspi_lat = qspi_lat & 0x3
        self.boot_halt = 1 if boot_halt else 0

        self.flash = W25Q128Flash(size_bytes=256 * 1024, dummy_clocks_eb=4)
        if flash_bytes:
            self.flash.load_bytes(0, flash_bytes)

        self.psram_a = APS6404LPsram(size_bytes=1024 * 1024, wait_cycles_eb=6)
        if psram_a_init:
            for addr, word in psram_a_init.items():
                self.psram_a.write_word32(int(addr), int(word))

        self.psram_b = APS6404LPsram(size_bytes=512 * 1024, wait_cycles_eb=6)

        self.uart = UartModel(clks_per_bit=208)
        self.spi = SpiSlaveModel(tx_bytes=spi_tx_bytes or [0x5A])
        self.i2c = I2cSlaveModel(addr7=i2c_addr)

        self._host_swclk = 0
        self._host_swdio = 1
        self._uart_rx = 1
        self._ext_irq = 0
        self._running = False
        self._bus_task = None
        self._uart_task = None

        self.swd = SwdHost(
            dut=dut,
            swclk_setter=self._set_swclk,
            swdio_setter=self._set_swdio,
            swdio_getter=self._get_swdio,
            clks_per_half_swclk=3,
        )

    def _ui_word(self, spi_miso: int = 0) -> int:
        return (
            ((self._host_swclk & 1) << 0)
            | ((spi_miso & 1) << 1)
            | ((self.boot_halt & 1) << 2)
            | ((self._uart_rx & 1) << 3)
            | ((self.qspi_lat & 0x3) << 4)
            | ((1 if self.pmod_mode else 0) << 6)
            | ((self._ext_irq & 1) << 7)
        )

    def _set_swclk(self, val: int) -> None:
        self._host_swclk = 1 if val else 0
        cur = _safe_int(self.dut.ui_in.value, 0)
        self.dut.ui_in.value = (cur & ~0x01) | self._host_swclk

    def _set_swdio(self, val: int) -> None:
        self._host_swdio = 1 if val else 0
        if not self.pmod_mode:
            cur = _safe_int(self.dut.uio_in.value, 0xFF)
            self.dut.uio_in.value = (cur & ~0x10) | (self._host_swdio << 4)

    def _get_swdio(self) -> int:
        uio_oe = _safe_int(self.dut.uio_oe.value, 0)
        if uio_oe & 0x10:
            return (_safe_int(self.dut.uio_out.value, 0) >> 4) & 1
        return self._host_swdio

    def set_boot_halt(self, halt: bool) -> None:
        self.boot_halt = 1 if halt else 0
        cur = _safe_int(self.dut.ui_in.value, 0)
        self.dut.ui_in.value = (cur & ~0x04) | (self.boot_halt << 2)

    async def reset(self, cycles: int = 6) -> None:
        self.dut.ena.value = 1
        self.dut.rst_n.value = 0
        self.dut.ui_in.value = self._ui_word(0)
        self.dut.uio_in.value = 0x70  # SWDIO, SCL, SDA pulled high
        await ClockCycles(self.dut.clk, cycles)

        # Verify clean async-reset idle outputs (no X)
        uo = int(self.dut.uo_out.value)
        assert (uo & 0x03) == 0x03, f"Expected Flash & PSRAM CS_n high at reset: {uo:#04x}"
        assert ((uo >> 2) & 1) == 0, "Expected QSPI_SCK low at reset"
        assert ((uo >> 4) & 1) == 1, "Expected UART_TX high at reset"
        assert ((uo >> 6) & 1) == 1, "Expected SPI_CS_n high at reset"

        self.dut.rst_n.value = 1
        await ClockCycles(self.dut.clk, 1)

    async def start(self) -> None:
        self._running = True
        self._bus_task = cocotb.start_soon(self._bus_loop())
        self._uart_task = cocotb.start_soon(
            self.uart.monitor_tx(
                self.dut, lambda: (_safe_int(self.dut.uo_out.value, 0xFF) >> 4) & 1
            )
        )

    async def stop(self) -> None:
        self._running = False
        self.uart.stop()
        if self._bus_task is not None:
            await self._bus_task
        if self._uart_task is not None:
            await self._uart_task

    async def _bus_loop(self) -> None:
        prev_qspi_sck = 0
        prev_spi_sck = 0
        i2c_slave_sda_in = 1

        while self._running:
            await RisingEdge(self.dut.clk)
            await ReadOnly()

            uo = _safe_int(self.dut.uo_out.value, 0xFF)
            uio_o = _safe_int(self.dut.uio_out.value, 0xFF)
            uio_oe = _safe_int(self.dut.uio_oe.value, 0x00)

            # 1. QSPI Flash + PSRAM A + PSRAM B
            if self.pmod_mode:
                cs0 = (uio_o >> 0) & 1 if (uio_oe & 0x01) else 1
                cs1 = (uio_o >> 6) & 1 if (uio_oe & 0x40) else 1
                cs2 = (uio_o >> 7) & 1 if (uio_oe & 0x80) else 1
                qsck = (uio_o >> 3) & 1 if (uio_oe & 0x08) else 0
                qio_out = (
                    (((uio_o >> 1) & 1) << 0)
                    | (((uio_o >> 2) & 1) << 1)
                    | (((uio_o >> 4) & 1) << 2)
                    | (((uio_o >> 5) & 1) << 3)
                )
            else:
                cs0 = (uo >> 0) & 1
                cs1 = (uo >> 1) & 1
                cs2 = (uio_o >> 7) & 1 if (uio_oe & 0x80) else 1
                qsck = (uo >> 2) & 1
                qio_out = uio_o & 0xF

            q_val = 0
            q_oe = 0
            for cs, dev in ((cs0, self.flash), (cs1, self.psram_a), (cs2, self.psram_b)):
                d_out, d_oe = dev.on_cs_change(cs)
                if qsck == 1 and prev_qspi_sck == 0:
                    d_out, d_oe = dev.on_sck_rise(qio_out)
                elif qsck == 0 and prev_qspi_sck == 1:
                    d_out, d_oe = dev.on_sck_fall()
                if d_oe:
                    q_val |= d_out & d_oe
                    q_oe |= d_oe
            prev_qspi_sck = qsck

            # 2. SPI peripheral slave (uo[3]=SCK, uo[5]=MOSI, uo[6]=CS_n -> ui[1]=MISO)
            spi_sck = (uo >> 3) & 1
            spi_mosi = (uo >> 5) & 1
            spi_cs_n = (uo >> 6) & 1
            spi_miso = self.spi.on_cs_change(spi_cs_n)
            if spi_sck == 1 and prev_spi_sck == 0:
                spi_miso = self.spi.on_sck_rise(spi_mosi)
            elif spi_sck == 0 and prev_spi_sck == 1:
                spi_miso = self.spi.on_sck_fall()
            prev_spi_sck = spi_sck

            # 3. I2C open-drain slave (Mode 0 only: uio[5]=SCL, uio[6]=SDA)
            if self.pmod_mode:
                scl_bus = 1
                sda_bus = 1
            else:
                scl_bus = 0 if (uio_oe & 0x20) else 1
                dut_sda_low = bool(uio_oe & 0x40)
                sda_bus = 0 if (dut_sda_low or (i2c_slave_sda_in == 0)) else 1
                i2c_slave_sda_in = self.i2c.step(scl_bus, sda_bus)

            # Compose next uio_in and ui_in after exiting ReadOnly
            if self.pmod_mode:
                next_uio_in = (
                    (((q_val >> 0) & 1) << 1)
                    | (((q_val >> 1) & 1) << 2)
                    | (((q_val >> 2) & 1) << 4)
                    | (((q_val >> 3) & 1) << 5)
                )
            else:
                next_uio_in = (
                    (q_val & 0xF)
                    | ((self._host_swdio & 1) << 4)
                    | ((scl_bus & 1) << 5)
                    | ((sda_bus & 1) << 6)
                )

            next_ui_in = self._ui_word(spi_miso)
            await Timer(1, unit="ps")
            self.dut.uio_in.value = next_uio_in
            self.dut.ui_in.value = next_ui_in

    # ---------------- SWD Helpers ----------------
    async def swd_connect(self) -> None:
        await self.swd.jtag_to_swd()
        idcode = await self.swd.read_dp(0x0)
        assert idcode == 0x11E70001, f"Unexpected DP IDCODE: {idcode:#010x}"
        await self.swd.write_dp(0x4, 0x50000000)

    async def swd_read_mem(self, addr: int) -> int:
        await self.swd.write_dp(0x8, 0x00000000)  # AP Bank 0
        await self.swd.write_ap(0x4, addr & 0xFFFFFF)
        await self.swd.read_ap(0x8)  # posted read
        return await self.swd.read_dp(0xC)

    async def swd_write_mem(self, addr: int, val: int) -> None:
        await self.swd.write_dp(0x8, 0x00000000)  # AP Bank 0
        await self.swd.write_ap(0x4, addr & 0xFFFFFF)
        await self.swd.write_ap(0x8, val & 0xFFFFFFFF)

    async def swd_read_telemetry(self) -> dict[str, int]:
        await self.swd.write_dp(0x8, 0x00000010)  # AP Bank 1
        cycle_cnt = await self.swd.read_ap(0x0)
        redex_cnt = await self.swd.read_ap(0x4)
        qspi_txn_cnt = await self.swd.read_ap(0x8)
        queue_depths = await self.swd.read_ap(0xC)
        return {
            "cycle_cnt": cycle_cnt,
            "redex_cnt": redex_cnt,
            "qspi_txn_cnt": qspi_txn_cnt,
            "queue_depths": queue_depths,
        }

    async def wait_done(self, max_cycles: int = 400_000) -> int:
        """Wait until core_busy|status_led (uo[7]) stays low across boot & execution."""
        # Wait past boot startup so status=1 is observed or at least 200 cycles pass
        await ClockCycles(self.dut.clk, 200)
        idle_streak = 0
        for cyc in range(200, max_cycles):
            await RisingEdge(self.dut.clk)
            uo = _safe_int(self.dut.uo_out.value, 0xFF)
            if ((uo >> 7) & 1) == 0:
                idle_streak += 1
                if idle_streak >= 64:
                    return cyc
            else:
                idle_streak = 0
        raise TimeoutError(f"MCU did not reach idle within {max_cycles} cycles")

    def i2c_wire_bytes(self) -> list[int]:
        out: list[int] = []
        for rw, addr7, data in self.i2c.txns:
            out.append(((addr7 & 0x7F) << 1) | (1 if rw == "R" else 0))
            out.extend(data)
        return out


@cocotb.test()
async def test_reset_and_swd_debug(dut):
    """Verify reset state, BOOT_HALT, SWD DP/AP, Flash/PSRAM inspection, single-step, and resume."""
    fx = json.loads((FIXTURES_DIR / "smoke_arith.json").read_text(encoding="utf-8"))
    flash_bytes = bytes.fromhex(fx["flash_hex"])

    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    board = BoardHarness(dut, flash_bytes=flash_bytes, boot_halt=True)
    await board.reset()
    await board.start()

    try:
        # Let boot finish and stall on BOOT_HALT
        await ClockCycles(dut.clk, 300)
        await board.swd_connect()

        # Verify AP IDR
        await board.swd.write_dp(0x8, 0x00000000)
        idr = await board.swd.read_ap(0xC)
        assert idr == 0x11E70010, f"AP IDR mismatch: {idr:#010x}"

        # Read Flash XIP magic via SWD Region 0x4
        magic = await board.swd_read_mem(0x400000)
        assert magic == 0x54454E49, f"SWD Flash magic mismatch: {magic:#010x}"

        # Write & read back PSRAM A via SWD Region 0x5
        await board.swd_write_mem(0x500400, 0xDEADBEEF)
        got_psram = await board.swd_read_mem(0x500400)
        assert got_psram == 0xDEADBEEF, f"SWD PSRAM A mismatch: {got_psram:#010x}"

        # Verify halted at reset before boot (core_busy=1, booted=0), then step to boot
        st0 = await board.swd_read_mem(0x000010)
        assert (st0 & 0x3) == 0x1, f"Expected halted-preboot DBG_STATUS (busy=1,booted=0): {st0:#010x}"

        # Step once to run Flash XIP boot while still BOOT_HALTed, then verify booted=1
        await board.swd.write_dp(0x8, 0x00000000)
        await board.swd.write_ap(0x0, 0x00000003)  # halt=1, step=1
        await ClockCycles(dut.clk, 300)
        st_booted = await board.swd_read_mem(0x000010)
        assert (st_booted & 0x2) == 0x2, f"Expected booted=1 after boot step: {st_booted:#010x}"

        # Execute 3 SWD single-steps while halted
        await board.swd.write_dp(0x8, 0x00000000)
        for _ in range(3):
            await board.swd.write_ap(0x0, 0x00000003)  # halt=1, step=1
            await ClockCycles(dut.clk, 200)

        telem_step = await board.swd_read_telemetry()
        assert telem_step["redex_cnt"] >= 1, f"Expected redex progress on step: {telem_step}"

        # Resume execution and verify final completion
        board.set_boot_halt(False)
        await board.swd.write_dp(0x8, 0x00000000)
        await board.swd.write_ap(0x0, 0x00000000)

        await board.wait_done(max_cycles=100_000)
        await ClockCycles(dut.clk, 2500)  # allow 115200-baud UART TX '!' to finish

        root0 = await board.swd_read_mem(0x000000)
        st1 = await board.swd_read_mem(0x000010)
        telem = await board.swd_read_telemetry()

        assert root0 == fx["expected_root"], f"ROOT mismatch: {root0:#x} != {fx['expected_root']:#x}"
        assert telem["redex_cnt"] == fx["expected_reductions"], (
            f"Redex mismatch: {telem['redex_cnt']} != {fx['expected_reductions']}"
        )
        assert (st1 & 0x1F) == 0x02, f"Unexpected status/errors at halt: {st1:#010x}"
        assert ((st1 >> 8) & 0xFFFF) == 0, f"Leaked hot nodes/wires: {st1:#010x}"
        assert board.uart.rx_bytes == fx["expected_uart_tx"], (
            f"UART mismatch: {board.uart.rx_bytes} != {fx['expected_uart_tx']}"
        )
    finally:
        await board.stop()


@cocotb.test()
async def test_pmod_mode_and_qspi_lat(dut):
    """Verify Mode 1 (ui[6]=1 mole99 QSPI Pmod) with qspi_lat=2 round-trip delay."""
    fx = json.loads((FIXTURES_DIR / "smoke_arith.json").read_text(encoding="utf-8"))
    flash_bytes = bytes.fromhex(fx["flash_hex"])

    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    board = BoardHarness(dut, flash_bytes=flash_bytes, pmod_mode=True, qspi_lat=2)
    await board.reset()
    await board.start()

    try:
        await board.wait_done(max_cycles=100_000)
        await ClockCycles(dut.clk, 2500)
        assert board.uart.rx_bytes == fx["expected_uart_tx"], (
            f"Mode 1 UART mismatch: {board.uart.rx_bytes} != {fx['expected_uart_tx']}"
        )
        assert board.flash.stats()["read_txns"] > 0
    finally:
        await board.stop()


@cocotb.test()
async def test_benchmarks_equivalence(dut):
    """Run all 5 compiled Vine benchmarks on tt_um_inet_mcu and verify golden equivalence."""
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    all_metrics: list[dict[str, object]] = []
    failures: list[str] = []

    for name in BENCHMARK_ORDER:
        fx = json.loads((FIXTURES_DIR / f"{name}.json").read_text(encoding="utf-8"))
        flash_bytes = bytes.fromhex(fx["flash_hex"])

        board = BoardHarness(
            dut,
            flash_bytes=flash_bytes,
            psram_a_init=fx.get("psram_init") or None,
            spi_tx_bytes=[0x5A],
            i2c_addr=0x50,
        )
        await board.reset()
        await board.start()

        try:
            await board.wait_done(max_cycles=400_000)
            # Ensure any in-flight UART byte finishes shifting at 208 clks/bit
            if fx.get("expected_uart_tx"):
                await ClockCycles(dut.clk, 2600)

            await board.swd_connect()
            root0 = await board.swd_read_mem(0x000000)
            status = await board.swd_read_mem(0x000010)
            hot_hits = await board.swd_read_mem(0x000014)
            psram_spills = await board.swd_read_mem(0x000018)
            telem = await board.swd_read_telemetry()

            fstats = board.flash.stats()
            pastats = board.psram_a.stats()
            pbstats = board.psram_b.stats()
            i2c_bytes = board.i2c_wire_bytes()
            row = {
                "name": name,
                "root": hex(root0),
                "expected_root": hex(fx["expected_root"]),
                "status": hex(status),
                "cycles": telem["cycle_cnt"],
                "reductions": telem["redex_cnt"],
                "expected_reductions": fx["expected_reductions"],
                "qspi_txns": telem["qspi_txn_cnt"],
                "hot_hits": hot_hits,
                "psram_spills": psram_spills,
                "flash_bytes_read": fstats["bytes_read"],
                "psram_a_bytes_r": pastats["bytes_read"],
                "psram_a_bytes_w": pastats["bytes_written"],
                "psram_b_bytes_r": pbstats["bytes_read"],
                "psram_b_bytes_w": pbstats["bytes_written"],
                "uart_rx": board.uart.rx_bytes,
                "spi_rx": board.spi.rx_bytes,
                "i2c_rx": i2c_bytes,
            }
            all_metrics.append(row)
            dut._log.info(f"[{name}] RESULT: {row}")

            if root0 != fx["expected_root"]:
                failures.append(f"[{name}] ROOT {root0:#x} != {fx['expected_root']:#x}")
            if telem["redex_cnt"] != fx["expected_reductions"]:
                failures.append(f"[{name}] redex {telem['redex_cnt']} != {fx['expected_reductions']}")
            if (status & 0x1F) != 0x02:
                failures.append(f"[{name}] status {status:#010x}")
            if ((status >> 8) & 0xFFFF) != 0:
                failures.append(f"[{name}] leaked hot slots {status:#010x}")
            if board.uart.rx_bytes != fx["expected_uart_tx"]:
                failures.append(f"[{name}] uart {board.uart.rx_bytes} != {fx['expected_uart_tx']}")
            if board.spi.rx_bytes != fx["expected_spi_tx"]:
                failures.append(f"[{name}] spi {board.spi.rx_bytes} != {fx['expected_spi_tx']}")
            if i2c_bytes != fx["expected_i2c_tx"]:
                failures.append(f"[{name}] i2c {i2c_bytes} != {fx['expected_i2c_tx']}")
        finally:
            await board.stop()

    Path("/tmp/tt_inet_rtl_metrics.json").write_text(
        json.dumps(all_metrics, indent=2), encoding="utf-8"
    )
    assert not failures, "Benchmark mismatches:\n" + "\n".join(failures)


@cocotb.test()
async def test_nordic_pio_pwm_xbar_dma(dut):
    """End-to-end top-level verification of XBAR, Timer/PWM + EasyDMA, PPI, PIO, and 4xINT8/Q15 Accel."""
    fx = json.loads((FIXTURES_DIR / "smoke_arith.json").read_text(encoding="utf-8"))
    flash_bytes = bytes.fromhex(fx["flash_hex"])

    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    board = BoardHarness(dut, flash_bytes=flash_bytes, boot_halt=True)
    await board.reset()
    await board.start()

    async def mmio_wr(dev: int, op: int, data: int = 0) -> None:
        addr = 0x700000 | ((dev & 0xF) << 8) | ((op & 0xF) << 2)
        await board.swd_write_mem(addr, data & 0xFF)

    async def mmio_rd(dev: int, op: int) -> int:
        addr = 0x700000 | ((dev & 0xF) << 8) | ((op & 0xF) << 2)
        return (await board.swd_read_mem(addr)) & 0xFFFF

    try:
        await ClockCycles(dut.clk, 100)
        await board.swd_connect()

        # ---------------- (a) GPIO Crossbar Remapping ----------------
        # Remap UART_TX (FN_UART_TX=1) onto uo[5] (out_sel2), transmit 0xA5 at div=7 (8 clks/bit)
        await mmio_wr(0, 5, 7)      # UART CFG_DIV_LO = 7
        await mmio_wr(0, 6, 0)      # UART CFG_DIV_HI = 0
        await mmio_wr(9, 2, 1)      # XBAR out_sel2 (uo[5]) <= FN_UART_TX(1)

        uart_xbar = UartModel(clks_per_bit=8)
        mon_uo5 = cocotb.start_soon(
            uart_xbar.monitor_tx(dut, lambda: (_safe_int(dut.uo_out.value, 0xFF) >> 5) & 1)
        )
        await mmio_wr(0, 0, 0xA5)   # UART TX 0xA5
        await ClockCycles(dut.clk, 120)
        uart_xbar.stop()
        await mon_uo5
        assert uart_xbar.rx_bytes == [0xA5], f"XBAR uo[5] UART_TX mismatch: {uart_xbar.rx_bytes}"
        await mmio_wr(9, 2, 3)      # restore uo[5] <= FN_SPI_MOS(3)

        # Remap PWM0 (FN_PWM0=7) onto uo[7] (out_sel4) & verify PWM duty ramp paced by EasyDMA
        await mmio_wr(9, 4, 7)      # XBAR out_sel4 (uo[7]) <= FN_PWM0(7)

        # ---------------- (b) Hardware PWM + EasyDMA Duty Ramp + PPI ----------------
        # PSRAM A: 3 duty words [4, 16, 30] at 0x001000..0x001008; descriptor at 0x001100
        for i, duty in enumerate((4, 16, 30)):
            await board.swd_write_mem(0x501000 + i * 4, duty)
        flags_pwm = (0x458 << 16) | 3  # SRC_PSRAM(0), DST_PWM0(3), src_inc(1), pace_en0(1), len=3
        await board.swd_write_mem(0x501100, 0x001000)
        await board.swd_write_mem(0x501104, 0)
        await board.swd_write_mem(0x501108, flags_pwm)
        await board.swd_write_mem(0x50110C, 0)

        # Configure Timer0 (TOP=39 -> 40 clks/period, init CC=2) & Timer1 (counter mode, auto_reload, en=1)
        await mmio_wr(4, 3, 39)     # T0 TOP_LO = 39
        await mmio_wr(4, 4, 0)      # T0 TOP_HI = 0
        await mmio_wr(4, 1, 2)      # T0 CC_LO = 2
        await mmio_wr(4, 2, 0)      # T0 CC_HI = 0
        await mmio_wr(4, 0, 0x02)   # T0 auto_reload=1, en=0
        await mmio_wr(4, 8 | 3, 255)
        await mmio_wr(4, 8 | 4, 0)
        await mmio_wr(4, 8 | 0, 0x07)  # T1 counter_mode=1, auto_reload=1, en=1

        # PPI Ch0: EVT_T0_CMP(0) -> TASK_T1_COUNT(2)
        await mmio_wr(5, 0, 0x80 | (2 << 4) | 0)
        # PPI Ch1: EVT_T0_PERIOD(1) -> TASK_INET_INJECT(7), pair = (ROOT(1), NUM(42))
        await mmio_wr(5, 4, 0x01)
        await mmio_wr(5, 5, 0x00)
        await mmio_wr(5, 6, 0x50)   # TAG_ROOT=5, idx=1 -> 0x500001
        await mmio_wr(5, 7, 42)
        await mmio_wr(5, 8, 0x00)
        await mmio_wr(5, 9, 0x30)   # TAG_NUM=3, val=42 -> 0x30002A
        await mmio_wr(5, 1, 0x80 | (7 << 4) | 1)

        # Start DMA (desc=0x001100) then start Timer0
        await mmio_wr(6, 0, 0x00)
        await mmio_wr(6, 1, 0x11)
        await mmio_wr(6, 2, 0x00)
        await mmio_wr(6, 3, 0x00)   # OP_START
        await mmio_wr(4, 6, 0x01)   # T0 CTRL start+clr

        pwm_samples = []
        for _ in range(220):
            await RisingEdge(dut.clk)
            pwm_samples.append((_safe_int(dut.uo_out.value, 0) >> 7) & 1)

        dma_st = await mmio_rd(6, 5)
        assert (dma_st & 0x200) != 0, f"PWM EasyDMA did not finish: status={dma_st:#x}"
        t1_cnt = await mmio_rd(4, 8 | 5)
        assert t1_cnt >= 3, f"PPI T0_CMP -> T1_COUNT failed: t1_cnt={t1_cnt}"
        root1 = await board.swd_read_mem(0x000004)
        assert root1 == 0x30002A, f"PPI TASK_INET_INJECT ROOT(1) mismatch: {root1:#010x}"

        # Compute high-cycle widths across consecutive rising-edge-delimited PWM periods on uo[7]
        widths = []
        cur_w = 0
        seen_rise = False
        for i in range(1, len(pwm_samples)):
            if pwm_samples[i - 1] == 0 and pwm_samples[i] == 1:
                if seen_rise:
                    widths.append(cur_w)
                cur_w = 1
                seen_rise = True
            elif seen_rise:
                cur_w += pwm_samples[i]
        assert len(set(widths)) >= 2 and max(widths) > min(widths), (
            f"Expected DMA-ramped PWM duty widths on uo[7], got widths={widths}"
        )
        await mmio_wr(4, 6, 0x02)   # stop Timer0
        await mmio_wr(9, 4, 13)     # restore uo[7] <= FN_STATUS(13)

        # ---------------- (c) RP2040 PIO 16x16 + DMA + Crossbar Loopback ----------------
        # Configure GPIO XBAR loopback: pio_in[0] (in_sel4) <- pio_pins_out[0] (source idx 5)
        await mmio_wr(9, 7, (4 << 4) | 5)
        # Program (wrap 0..3):
        #   0: OUT PINS, 1 [1] (0x6101)
        #   1: IN  PINS, 1     (0x4001)
        #   2: JMP !OSRE, 0    (0x00E0)
        #   3: IRQ 0 side 2    (0xD000)
        prog = [0x6101, 0x4001, 0x00E0, 0xD000]
        await mmio_wr(8, 1, 0x00)   # wr_addr = 0
        for insn in prog:
            await mmio_wr(8, 2, insn & 0xFF)
            await mmio_wr(8, 3, (insn >> 8) & 0xFF)
        await mmio_wr(8, 4, (3 << 4) | 0)  # wrap_top=3, wrap_bot=0
        await mmio_wr(8, 5, 0)             # clk_div = 0
        await mmio_wr(8, 6, 0)
        # CFG: autopull=1(6), autopush=1(5), shift_r_out=1(4), shift_r_in=1(3), sideset=1(2), clr(1), rst_sm(0)
        await mmio_wr(8, 0, 0x7F)
        # Enable SM with DMA streaming 0xA55A from PSRAM A 0x001200 -> DST_PIO(2)
        await board.swd_write_mem(0x501200, 0x0000A55A)
        await board.swd_write_mem(0x501300, 0x001200)
        await board.swd_write_mem(0x501304, 0)
        await board.swd_write_mem(0x501308, ((0 << 0) | (2 << 3)) << 16 | 1)  # SRC_PSRAM->DST_PIO, len=1
        await board.swd_write_mem(0x50130C, 0)
        await mmio_wr(5, 11, 1)     # clear PPI sticky events
        await mmio_wr(8, 0, 0xFC)   # sm_en=1, autopull=1, autopush=1, shift_r_out=1, shift_r_in=1, sideset=1
        await mmio_wr(6, 0, 0x00)
        await mmio_wr(6, 1, 0x13)
        await mmio_wr(6, 2, 0x00)
        await mmio_wr(6, 3, 0x00)   # start DMA
        await ClockCycles(dut.clk, 120)
        rx_word = await mmio_rd(8, 9)  # OP_RX_POP
        await mmio_wr(8, 0, 0x7C)   # disable SM
        ppi_evts = await mmio_rd(5, 11)
        assert rx_word == 0xA55A, f"RP2040 PIO DMA loopback mismatch: {rx_word:#06x}"
        assert (ppi_evts & (1 << 9)) != 0, f"Expected EVT_PIO_IRQ(9) sticky in PPI: {ppi_evts:#x}"

        # ---------------- (d) Chained Linked-List DMA -> 4xINT8 SIMD & Q15 Accel ----------------
        # Vectors: A0=[2,-3,4,5]=0x0504FD02, B0=[10,20,-5,2]=0x02FB140A -> -50
        #          A1=[4,4,4,4]=0x04040404,  B1=[10,10,10,10]=0x0A0A0A0A -> +160 (total = 110)
        await board.swd_write_mem(0x502000, 0x0504FD02)
        await board.swd_write_mem(0x502004, 0x02FB140A)
        await board.swd_write_mem(0x502008, 0x04040404)
        await board.swd_write_mem(0x50200C, 0x0A0A0A0A)
        await board.swd_write_mem(0x502020, 0xDEADBEEF)
        # Desc0 (0x002100): SRC_PSRAM(0) -> DST_ACCEL_AB(5), src_inc(0x40), chain_en(0x200), len=4
        await board.swd_write_mem(0x502100, 0x002000)
        await board.swd_write_mem(0x502104, 0)
        await board.swd_write_mem(0x502108, (0x268 << 16) | 4)
        await board.swd_write_mem(0x50210C, 0x002110)
        # Desc1 (0x002110): SRC_ACCEL(3) -> DST_PSRAM(0), dst_inc(0x80), len=1
        await board.swd_write_mem(0x502110, 0)
        await board.swd_write_mem(0x502114, 0x002020)
        await board.swd_write_mem(0x502118, (0x083 << 16) | 1)
        await board.swd_write_mem(0x50211C, 0)

        await mmio_wr(7, 0, 0x00)   # ACCEL CFG: 4xINT8, no ReLU, no auto_clr, shift=0
        await mmio_wr(7, 13, 0)     # ACCEL CLEAR
        await mmio_wr(5, 11, 1)     # clear PPI sticky events
        await mmio_wr(6, 0, 0x00)
        await mmio_wr(6, 1, 0x21)
        await mmio_wr(6, 2, 0x00)
        await mmio_wr(6, 3, 0x00)   # start chained DMA
        for _ in range(40):
            dma_st = await mmio_rd(6, 5)
            if (dma_st & 0x200) != 0:
                break
        else:
            raise TimeoutError(f"Chained Accel DMA timeout: {dma_st:#x}")
        got_dot = await board.swd_read_mem(0x502020)
        ppi_evts2 = await mmio_rd(5, 11)
        assert got_dot == 110, f"DMA->ACCEL 4xINT8 dot product mismatch: {got_dot} != 110"
        assert (ppi_evts2 & (1 << 8)) != 0, f"Expected EVT_DMA_CHAIN_DONE(8): {ppi_evts2:#x}"

        # Q15 MAC + ReLU + shift=15 check via MMIO: 0.5 (0x4000) * 0.5 (0x4000) >> 15 = 0.25 (0x2000)
        await mmio_wr(7, 13, 0)
        await mmio_wr(7, 0, (15 << 3) | 0x03)  # shift=15, relu=1, mode_q15=1
        await mmio_wr(7, 1, 0x00); await mmio_wr(7, 2, 0x40); await mmio_wr(7, 3, 0x00); await mmio_wr(7, 4, 0x00)
        await mmio_wr(7, 5, 0x00); await mmio_wr(7, 6, 0x40); await mmio_wr(7, 7, 0x00); await mmio_wr(7, 8, 0x00)
        q15_out = await mmio_rd(7, 12)      # OP_READ_QUANT(int8_mode=0)
        assert q15_out == 0x2000, f"ACCEL Q15 MAC mismatch: {q15_out:#06x} != 0x2000"
    finally:
        await board.stop()
