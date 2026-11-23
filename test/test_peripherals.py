"""Cocotb unit tests verifying unified reconfigurable inet_sercom (UART, SPI, I2C + live switching)."""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from models import I2cSlaveModel, SpiSlaveModel, UartModel

MODE_UART = 0
MODE_SPI = 1
MODE_I2C = 2


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.req_valid.value = 0
    dut.req_mode.value = MODE_UART
    dut.req_mode_auto.value = 1
    dut.req_op.value = 0
    dut.req_data.value = 0
    dut.resp_ready.value = 1
    dut.uart_rx.value = 1
    dut.spi_miso.value = 0
    dut.ext_scl_low.value = 0
    dut.ext_sda_low.value = 0

    await ClockCycles(dut.clk, 5)
    assert int(dut.uart_tx.value) == 1
    assert int(dut.spi_cs_n.value) == 1
    assert int(dut.spi_sck.value) == 0
    assert int(dut.scl_drive_low.value) == 0
    assert int(dut.sda_drive_low.value) == 0
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def xfer_req(
    dut, mode: int, op: int, data: int, auto: int = 1, timeout_cycles: int = 5000
):
    dut.req_mode.value = mode
    dut.req_mode_auto.value = auto
    dut.req_op.value = op
    dut.req_data.value = data
    dut.req_valid.value = 1

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if int(dut.req_ready.value) == 1:
            break
    else:
        raise TimeoutError(f"SERCOM req_ready timeout (mode={mode}, op={op})")

    dut.req_valid.value = 0

    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if int(dut.resp_valid.value) == 1:
            return int(dut.resp_status.value), int(dut.resp_data.value)
    raise TimeoutError(f"SERCOM resp_valid timeout (mode={mode}, op={op})")


@cocotb.test()
async def test_uart_tx_rx(dut):
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    await reset_dut(dut)

    await xfer_req(dut, MODE_UART, 5, 7)  # U_CFG_DIV_LO = 7 (8 clks/bit)
    await xfer_req(dut, MODE_UART, 6, 0)  # U_CFG_DIV_HI

    uart = UartModel(clks_per_bit=8)
    mon = cocotb.start_soon(uart.monitor_tx(dut, lambda: int(dut.uart_tx.value)))

    msg = [0x48, 0x69, 0x21]  # "Hi!"
    for b in msg:
        await xfer_req(dut, MODE_UART, 0, b)  # U_TX
    await xfer_req(dut, MODE_UART, 4, 0)  # U_TX_WAIT
    await ClockCycles(dut.clk, 20)
    uart.stop()
    await mon
    assert uart.rx_bytes == msg, f"UART TX mismatch: {uart.rx_bytes}"

    sender = cocotb.start_soon(
        uart.send_byte(dut, lambda v: setattr(dut.uart_rx, "value", v), 0xA7)
    )
    status, rx_byte = await xfer_req(dut, MODE_UART, 1, 0)  # U_RX_BLOCK
    await sender
    assert rx_byte == 0xA7, f"UART RX mismatch: {rx_byte:#x}"
    assert (status & 0x0C) == 0, f"Unexpected UART err status: {status:#x}"


@cocotb.test()
async def test_spi_full_duplex(dut):
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    await reset_dut(dut)

    await xfer_req(dut, MODE_SPI, 4, 1)  # S_CFG_DIV = 1
    slave = SpiSlaveModel(tx_bytes=[0x5A, 0xC3])
    task = cocotb.start_soon(
        slave.run(
            dut,
            cs_getter=lambda: int(dut.spi_cs_n.value),
            sck_getter=lambda: int(dut.spi_sck.value),
            mosi_getter=lambda: int(dut.spi_mosi.value),
            miso_setter=lambda v: setattr(dut.spi_miso, "value", v),
        )
    )

    await xfer_req(dut, MODE_SPI, 1, 0)  # S_CS_LOW
    _, r0 = await xfer_req(dut, MODE_SPI, 0, 0x12)
    _, r1 = await xfer_req(dut, MODE_SPI, 0, 0x34)
    await xfer_req(dut, MODE_SPI, 2, 0)  # S_CS_HIGH
    await ClockCycles(dut.clk, 4)
    slave.stop()
    await task

    assert [r0, r1] == [0x5A, 0xC3], f"SPI MISO mismatch: {[r0, r1]}"
    assert slave.rx_bytes == [0x12, 0x34], f"SPI MOSI mismatch: {slave.rx_bytes}"


@cocotb.test()
async def test_i2c_write_and_read(dut):
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    await reset_dut(dut)

    await xfer_req(dut, MODE_I2C, 6, 3)  # I_CFG_DIV = 3
    slave = I2cSlaveModel(addr7=0x50)
    task = cocotb.start_soon(
        slave.run(
            dut,
            scl_getter=lambda: int(dut.scl_bus.value),
            sda_getter=lambda: int(dut.sda_bus.value),
            sda_setter=lambda v: setattr(dut.ext_sda_low, "value", 1 if v == 0 else 0),
        )
    )

    await xfer_req(dut, MODE_I2C, 0, 0)  # I_START
    for b in (0xA0, 0x10, 0x77, 0x88):
        st, _ = await xfer_req(dut, MODE_I2C, 2, b)  # I_WRITE_BYTE
        assert (st & 0x01) == 1, f"I2C NACK on write {b:#x} (status={st:#x})"
    await xfer_req(dut, MODE_I2C, 1, 0)  # I_STOP

    await xfer_req(dut, MODE_I2C, 0, 0)  # I_START
    for b in (0xA0, 0x10):
        st, _ = await xfer_req(dut, MODE_I2C, 2, b)
        assert (st & 0x01) == 1
    await xfer_req(dut, MODE_I2C, 0, 0)  # Repeated I_START
    st, _ = await xfer_req(dut, MODE_I2C, 2, 0xA1)
    assert (st & 0x01) == 1
    _, b0 = await xfer_req(dut, MODE_I2C, 3, 0)  # I_READ_ACK
    _, b1 = await xfer_req(dut, MODE_I2C, 4, 0)  # I_READ_NACK
    await xfer_req(dut, MODE_I2C, 1, 0)  # I_STOP

    await ClockCycles(dut.clk, 10)
    slave.stop()
    await task

    assert (b0, b1) == (0x77, 0x88), f"I2C read mismatch: {(b0, b1)}"
    assert slave.regs.get(0x10) == 0x77 and slave.regs.get(0x11) == 0x88


@cocotb.test()
async def test_sercom_runtime_mode_switching(dut):
    """Verify UART -> SPI -> I2C -> UART live switching without losing UART TX integrity."""
    cocotb.start_soon(Clock(dut.clk, 40, unit="ns").start())
    await reset_dut(dut)

    await xfer_req(dut, MODE_UART, 5, 7)
    await xfer_req(dut, MODE_UART, 6, 0)
    await xfer_req(dut, MODE_SPI, 4, 1)
    await xfer_req(dut, MODE_I2C, 6, 3)

    uart = UartModel(clks_per_bit=8)
    uart_mon = cocotb.start_soon(uart.monitor_tx(dut, lambda: int(dut.uart_tx.value)))
    spi = SpiSlaveModel(tx_bytes=[0xB6])
    spi_task = cocotb.start_soon(
        spi.run(
            dut,
            cs_getter=lambda: int(dut.spi_cs_n.value),
            sck_getter=lambda: int(dut.spi_sck.value),
            mosi_getter=lambda: int(dut.spi_mosi.value),
            miso_setter=lambda v: setattr(dut.spi_miso, "value", v),
        )
    )
    i2c = I2cSlaveModel(addr7=0x50)
    i2c_task = cocotb.start_soon(
        i2c.run(
            dut,
            scl_getter=lambda: int(dut.scl_bus.value),
            sda_getter=lambda: int(dut.sda_bus.value),
            sda_setter=lambda v: setattr(dut.ext_sda_low, "value", 1 if v == 0 else 0),
        )
    )

    # UART byte while switching immediately to SPI: mode switch must wait for UART TX stop bit
    await xfer_req(dut, MODE_UART, 0, 0x4B)  # 'K'
    await xfer_req(dut, MODE_SPI, 1, 0)     # S_CS_LOW (auto switches after TX finishes)
    assert int(dut.cur_mode.value) == MODE_SPI
    _, spi_rx = await xfer_req(dut, MODE_SPI, 0, 0x3C)
    await xfer_req(dut, MODE_SPI, 2, 0)     # S_CS_HIGH

    # Switch immediately to I2C write
    await xfer_req(dut, MODE_I2C, 0, 0)     # I_START
    assert int(dut.cur_mode.value) == MODE_I2C
    for b in (0xA0, 0x05, 0x99):
        st, _ = await xfer_req(dut, MODE_I2C, 2, b)
        assert (st & 0x01) == 1
    await xfer_req(dut, MODE_I2C, 1, 0)     # I_STOP

    # Switch back to UART
    await xfer_req(dut, MODE_UART, 0, 0x4F)  # 'O'
    await xfer_req(dut, MODE_UART, 4, 0)     # U_TX_WAIT
    assert int(dut.cur_mode.value) == MODE_UART
    await ClockCycles(dut.clk, 20)

    uart.stop()
    spi.stop()
    i2c.stop()
    await uart_mon
    await spi_task
    await i2c_task

    assert uart.rx_bytes == [0x4B, 0x4F], f"UART switch bytes mismatch: {uart.rx_bytes}"
    assert spi_rx == 0xB6 and spi.rx_bytes == [0x3C], f"SPI switch mismatch: {spi_rx:#x}, {spi.rx_bytes}"
    assert i2c.regs.get(0x05) == 0x99, f"I2C reg 0x05 mismatch: {i2c.regs}"
