"""Cocotb unit tests verifying inet_qspi_ctrl and inet_swd against behavioral models."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

from models import APS6404LPsram, SwdHost, W25Q128Flash


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.qspi_lat.value = 0
    dut.cmd_valid.value = 0
    dut.cmd_dev.value = 0
    dut.cmd_write.value = 0
    dut.cmd_addr.value = 0
    dut.cmd_words.value = 0
    dut.wdata.value = 0
    dut.wdata_valid.value = 0
    dut.rdata_ready.value = 1
    dut.qspi_io_in.value = 0

    dut.swclk_in.value = 0
    dut.host_swdio_in.value = 1
    dut.boot_halt.value = 0
    dut.core_halted.value = 0
    dut.core_busy.value = 1
    dut.active_tiles.value = 4
    dut.cycle_cnt.value = 0x00001234
    dut.redex_cnt.value = 0x00000567
    dut.qspi_txn_cnt.value = 0x00000089
    dut.queue_depths.value = 0x04030201
    dut.dbg_mem_ready.value = 0
    dut.dbg_mem_rdata.value = 0

    await ClockCycles(dut.clk, 5)
    assert int(dut.qspi_cs0_n.value) == 1
    assert int(dut.qspi_cs1_n.value) == 1
    assert int(dut.qspi_cs2_n.value) == 1
    assert int(dut.qspi_sck.value) == 0
    assert int(dut.swdio_oe.value) == 0
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def run_qspi_devices(dut, flash, psram_a, psram_b, stop_flag):
    prev_sck = 0
    while not stop_flag["stop"]:
        await RisingEdge(dut.clk)
        await ReadOnly()
        cs0 = int(dut.qspi_cs0_n.value)
        cs1 = int(dut.qspi_cs1_n.value)
        cs2 = int(dut.qspi_cs2_n.value)
        sck = int(dut.qspi_sck.value)
        io_out_dut = int(dut.qspi_io_out.value)
        io_oe_dut = int(dut.qspi_io_oe.value)
        io_bus = io_out_dut & io_oe_dut

        out_val = 0
        out_oe = 0
        for cs, dev in ((cs0, flash), (cs1, psram_a), (cs2, psram_b)):
            d_out, d_oe = dev.on_cs_change(cs)
            if sck == 1 and prev_sck == 0:
                d_out, d_oe = dev.on_sck_rise(io_bus)
            elif sck == 0 and prev_sck == 1:
                d_out, d_oe = dev.on_sck_fall()
            if d_oe:
                out_val |= d_out & d_oe
                out_oe |= d_oe
        prev_sck = sck
        await Timer(1, unit="ps")
        dut.qspi_io_in.value = (out_val & out_oe) | (io_bus & ~out_oe)


async def qspi_read_burst(dut, dev: int, addr: int, count: int) -> list[int]:
    while int(dut.cmd_ready.value) != 1:
        await RisingEdge(dut.clk)
    dut.cmd_dev.value = dev
    dut.cmd_write.value = 0
    dut.cmd_addr.value = addr
    dut.cmd_words.value = count - 1
    dut.cmd_valid.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_valid.value = 0

    words = []
    for _ in range(500):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.rdata_valid.value) == 1:
            words.append(int(dut.rdata.value))
            if len(words) == count:
                break
    else:
        raise TimeoutError(f"QSPI read timeout (got {len(words)}/{count})")
    while int(dut.qspi_busy.value) == 1:
        await RisingEdge(dut.clk)
    return words


async def qspi_write_burst(dut, dev: int, addr: int, words: list[int]) -> None:
    while int(dut.cmd_ready.value) != 1:
        await RisingEdge(dut.clk)
    dut.cmd_dev.value = dev
    dut.cmd_write.value = 1
    dut.cmd_addr.value = addr
    dut.cmd_words.value = len(words) - 1
    dut.cmd_valid.value = 1
    await RisingEdge(dut.clk)
    dut.cmd_valid.value = 0

    idx = 0
    for _ in range(500):
        dut.wdata.value = words[idx]
        dut.wdata_valid.value = 1 if idx < len(words) else 0
        await RisingEdge(dut.clk)
        await ReadOnly()
        hit = int(dut.wdata_ready.value) == 1
        await Timer(1, unit="ps")
        if hit:
            idx += 1
            if idx == len(words):
                dut.wdata_valid.value = 0
                break
    else:
        raise TimeoutError("QSPI write wdata_ready timeout")

    while int(dut.qspi_busy.value) == 1:
        await RisingEdge(dut.clk)


@cocotb.test()
async def test_qspi_flash_and_psram(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_dut(dut)

    flash = W25Q128Flash(size_bytes=65536, dummy_clocks_eb=4)
    psram_a = APS6404LPsram(size_bytes=1024 * 1024, wait_cycles_eb=6)
    psram_b = APS6404LPsram(size_bytes=1024 * 1024, wait_cycles_eb=6)

    flash_words = [0x54454E49, 0x00040001, 0x00800001, 0x0030000A]
    flash.load_bytes(0x000100, b"".join(w.to_bytes(4, "little") for w in flash_words))

    stop_flag = {"stop": False}
    qtask = cocotb.start_soon(run_qspi_devices(dut, flash, psram_a, psram_b, stop_flag))

    # Flash XIP 0xEB read with qspi_lat=0 and qspi_lat=2
    dut.qspi_lat.value = 0
    got0 = await qspi_read_burst(dut, 0, 0x000100, 4)
    assert got0 == flash_words, f"Flash lat=0 mismatch: {[hex(x) for x in got0]}"

    dut.qspi_lat.value = 2
    got2 = await qspi_read_burst(dut, 0, 0x000100, 4)
    assert got2 == flash_words, f"Flash lat=2 mismatch: {[hex(x) for x in got2]}"

    # PSRAM A 0x38 quad write + 0xEB quad read
    dut.qspi_lat.value = 0
    pa_words = [0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0x00A55A00]
    await qspi_write_burst(dut, 1, 0x001000, pa_words)
    for i, w in enumerate(pa_words):
        assert psram_a.read_word32(0x001000 + 4 * i) == w
    got_pa = await qspi_read_burst(dut, 1, 0x001000, 4)
    assert got_pa == pa_words, f"PSRAM A readback mismatch: {[hex(x) for x in got_pa]}"

    # PSRAM B 0x38 write + 0xEB read
    await qspi_write_burst(dut, 2, 0x002004, [0xA55AF00D])
    assert psram_b.read_word32(0x002004) == 0xA55AF00D
    got_pb = await qspi_read_burst(dut, 2, 0x002004, 1)
    assert got_pb == [0xA55AF00D]

    stop_flag["stop"] = True
    await qtask


@cocotb.test()
async def test_swd_target(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    await reset_dut(dut)

    # Simple 3-cycle debug memory responder
    mem = {}
    stop_flag = {"stop": False}

    async def dbg_mem_responder():
        while not stop_flag["stop"]:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.dbg_mem_req.value) == 1:
                addr = int(dut.dbg_mem_addr.value)
                we = int(dut.dbg_mem_we.value)
                wval = int(dut.dbg_mem_wdata.value)
                await ClockCycles(dut.clk, 3)
                if we:
                    mem[addr] = wval
                else:
                    dut.dbg_mem_rdata.value = mem.get(addr, 0)
                dut.dbg_mem_ready.value = 1
                await RisingEdge(dut.clk)
                dut.dbg_mem_ready.value = 0

    mtask = cocotb.start_soon(dbg_mem_responder())

    swd = SwdHost(
        dut=dut,
        swclk_setter=lambda v: setattr(dut.swclk_in, "value", v),
        swdio_setter=lambda v: setattr(dut.host_swdio_in, "value", v),
        swdio_getter=lambda: int(dut.swdio_bus.value),
        clks_per_half_swclk=4,
    )

    await swd.jtag_to_swd()
    idcode = await swd.read_dp(0x0)
    assert idcode == 0x11E70001, f"DP IDCODE mismatch: {idcode:#010x}"

    await swd.write_dp(0x4, 0x50000000)
    ctrl_stat = await swd.read_dp(0x4)
    assert (ctrl_stat & 0xF0000000) == 0xF0000000, f"CTRL/STAT mismatch: {ctrl_stat:#010x}"

    # Select AP Bank 0 and read IDR
    await swd.write_dp(0x8, 0x00000000)
    idr = await swd.read_ap(0xC)
    assert idr == 0x11E70010, f"AP IDR mismatch: {idr:#010x}"

    # Halt core via DBG_CTRL[0]
    await swd.write_ap(0x0, 0x00000001)
    assert int(dut.dbg_halt_req.value) == 1

    # Write memory via AP DBG_ADDR + DBG_DATA, then posted read + RDBUFF
    await swd.write_ap(0x4, 0x00000100)
    await swd.write_ap(0x8, 0xCAFEBABE)
    await swd.read_ap(0x8)  # triggers posted read
    rdbuff = await swd.read_dp(0xC)
    assert rdbuff == 0xCAFEBABE, f"SWD memory readback mismatch: {rdbuff:#010x}"

    # Select AP Bank 1 and read telemetry counters
    await swd.write_dp(0x8, 0x00000010)
    assert await swd.read_ap(0x0) == 0x00001234
    assert await swd.read_ap(0x4) == 0x00000567
    assert await swd.read_ap(0x8) == 0x00000089
    assert await swd.read_ap(0xC) == 0x04030201

    stop_flag["stop"] = True
    await mtask
