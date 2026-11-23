"""Cocotb verification of inet_io_bridge concurrent multi-peripheral execution."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge, Timer

from models import I2cSlaveModel, SpiSlaveModel, UartModel


async def send_io_req(dut, dev: int, op: int, data: int, cont: int, timeout: int = 2000) -> None:
    await Timer(1, unit="ps")
    dut.io_req_dev.value = dev
    dut.io_req_val.value = ((op & 0xF) << 8) | (data & 0xFF)
    dut.io_req_cont.value = cont & 0xFFFFFF
    dut.io_req_valid.value = 1

    for _ in range(timeout):
        await RisingEdge(dut.clk)
        if int(dut.io_req_ready.value) == 1:
            break
    else:
        raise TimeoutError(f"io_req_ready timeout for dev={dev} op={op}")

    await Timer(1, unit="ps")
    dut.io_req_valid.value = 0


async def wait_io_out(dut, timeout: int = 5000) -> tuple[int, int]:
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(dut.io_out_valid.value) == 1:
            a = int(dut.io_out_a.value)
            b = int(dut.io_out_b.value)
            await Timer(1, unit="ps")
            return a, b
    raise TimeoutError("io_out_valid timeout")


async def io_call(dut, dev: int, op: int, data: int, cont: int) -> tuple[int, int]:
    await send_io_req(dut, dev, op, data, cont)
    a, b = await wait_io_out(dut)
    assert a == cont, f"Continuation mismatch: got {a:#x}, expected {cont:#x}"
    assert ((b >> 20) & 0xF) == 0x3, f"Expected TAG_NUM=3 in {b:#x}"
    return (b >> 8) & 0xFF, b & 0xFF


@cocotb.test()
async def test_io_bridge_concurrent(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())

    dut.rst_n.value = 0
    dut.io_req_valid.value = 0
    dut.io_req_dev.value = 0
    dut.io_req_val.value = 0
    dut.io_req_cont.value = 0
    dut.io_out_ready.value = 1
    dut.cycle_cnt.value = 0x12345678
    dut.ext_irq.value = 1
    dut.uart_rx.value = 1
    dut.spi_miso.value = 0
    dut.ext_scl_low.value = 0
    dut.ext_sda_low.value = 0

    await ClockCycles(dut.clk, 5)
    assert int(dut.io_out_valid.value) == 0
    assert int(dut.status_led.value) == 0
    assert int(dut.uart_tx.value) == 1
    assert int(dut.spi_cs_n.value) == 1
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)

    # Configure fast baud/clock dividers and assert SPI CS_low
    await io_call(dut, 0, 5, 7, 0x100001)   # UART CFG_DIV_LO = 7 (8 clks/bit)
    await io_call(dut, 0, 6, 0, 0x100002)   # UART CFG_DIV_HI = 0
    await io_call(dut, 1, 4, 1, 0x100003)   # SPI CFG_DIV = 1
    await io_call(dut, 1, 1, 0, 0x100004)   # SPI CS_LOW
    await io_call(dut, 2, 6, 3, 0x100005)   # I2C CFG_DIV = 3

    uart = UartModel(clks_per_bit=8)
    spi = SpiSlaveModel(tx_bytes=[0xA9])
    i2c = I2cSlaveModel(addr7=0x50)

    utask = cocotb.start_soon(uart.monitor_tx(dut, lambda: int(dut.uart_tx.value)))
    stask = cocotb.start_soon(
        spi.run(
            dut,
            cs_getter=lambda: int(dut.spi_cs_n.value),
            sck_getter=lambda: int(dut.spi_sck.value),
            mosi_getter=lambda: int(dut.spi_mosi.value),
            miso_setter=lambda v: setattr(dut.spi_miso, "value", v),
        )
    )
    itask = cocotb.start_soon(
        i2c.run(
            dut,
            scl_getter=lambda: int(dut.scl_bus.value),
            sda_getter=lambda: int(dut.sda_bus.value),
            sda_setter=lambda v: setattr(dut.ext_sda_low, "value", 1 if v == 0 else 0),
        )
    )

    # Background collector to record exact completion order across devices
    completions: list[tuple[int, int]] = []
    stop_collect = {"stop": False}

    async def collector():
        while not stop_collect["stop"]:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(dut.io_out_valid.value) == 1:
                completions.append((int(dut.io_out_a.value), int(dut.io_out_b.value)))
            await Timer(1, unit="ps")

    ctask = cocotb.start_soon(collector())

    # Launch 4 concurrent operations back-to-back without waiting between them:
    # UART TX (fast resp), SPI XFER (medium), I2C START (slow), SYS CYCLE_LO (1 cycle)
    await send_io_req(dut, 0, 0, 0x55, 0x500001)  # UART TX 0x55
    await send_io_req(dut, 1, 0, 0x3C, 0x500002)  # SPI XFER 0x3C -> expect 0xA9
    await send_io_req(dut, 2, 0, 0x00, 0x500003)  # I2C START
    await send_io_req(dut, 3, 1, 0x00, 0x500004)  # SYS CYCLE_LO

    for _ in range(500):
        if len(completions) >= 4:
            break
        await ClockCycles(dut.clk, 1)
    else:
        raise TimeoutError(f"Expected 4 concurrent completions, got {completions}")

    order = [a for a, _ in completions]
    by_cont = {a: b for a, b in completions}

    # Prove zero head-of-line blocking: SYS completes before I2C/SPI finish
    assert order.index(0x500004) < order.index(0x500002), f"Expected SYS before SPI: {order}"
    assert order.index(0x500004) < order.index(0x500003), f"Expected SYS before I2C: {order}"
    assert (by_cont[0x500002] & 0xFF) == 0xA9, f"SPI RX mismatch: {by_cont[0x500002]:#x}"
    assert (by_cont[0x500004] & 0xFFFF) == 0x5678, f"SYS_CYCLE_LO mismatch: {by_cont[0x500004]:#x}"

    stop_collect["stop"] = True
    await ctask

    # Verify SYS_STATUS_LED and SYS_GPIO_RD
    _, led_d = await io_call(dut, 3, 4, 1, 0x500005)
    assert led_d == 1 and int(dut.status_led.value) == 1
    _, gpio_d = await io_call(dut, 3, 3, 0, 0x500006)
    assert gpio_d == 1

    # Complete the started I2C write transaction: addr 0xA0, reg 0x04, data 0x99, STOP
    for b in (0xA0, 0x04, 0x99):
        st, _ = await io_call(dut, 2, 2, b, 0x600000 + b)
        assert (st & 0x01) == 1, f"Expected I2C ACK on {b:#x}, status={st:#x}"
    await io_call(dut, 2, 1, 0, 0x6000FF)  # I2C STOP
    await io_call(dut, 1, 2, 0, 0x600100)  # SPI CS_HIGH
    await io_call(dut, 0, 4, 0, 0x600101)  # UART TX_WAIT

    await ClockCycles(dut.clk, 20)
    uart.stop()
    spi.stop()
    i2c.stop()
    await utask
    await stask
    await itask

    assert uart.rx_bytes == [0x55], f"UART capture mismatch: {uart.rx_bytes}"
    assert spi.rx_bytes == [0x3C], f"SPI capture mismatch: {spi.rx_bytes}"
    assert i2c.regs.get(0x04) == 0x99, f"I2C reg mismatch: {i2c.regs}"
