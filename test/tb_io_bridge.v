`default_nettype none
`timescale 1ns / 1ps

module tb_io_bridge ();
    reg clk;
    reg rst_n;

    reg         io_req_valid;
    wire        io_req_ready;
    reg  [3:0]  io_req_dev;
    reg  [15:0] io_req_val;
    reg  [23:0] io_req_cont;

    wire        io_out_valid;
    reg         io_out_ready;
    wire [23:0] io_out_a;
    wire [23:0] io_out_b;

    reg  [31:0] cycle_cnt;
    reg         ext_irq;
    wire        status_led;
    wire        io_busy;

    reg         uart_rx;
    wire        uart_tx;
    wire        spi_sck;
    wire        spi_mosi;
    reg         spi_miso;
    wire        spi_cs_n;

    reg         ext_scl_low;
    reg         ext_sda_low;
    wire        pin_uo_7;
    wire        pin_uio_5_out, pin_uio_5_oe;
    wire        pin_uio_6_out, pin_uio_6_oe;
    wire        i2c_scl_drive_low = pin_uio_5_oe && !pin_uio_5_out;
    wire        i2c_sda_drive_low = pin_uio_6_oe && !pin_uio_6_out;

    wire scl_bus = !(i2c_scl_drive_low | ext_scl_low);
    wire sda_bus = !(i2c_sda_drive_low | ext_sda_low);

    inet_io_bridge u_bridge (
        .clk               (clk),
        .rst_n             (rst_n),
        .io_req_valid      (io_req_valid),
        .io_req_ready      (io_req_ready),
        .io_req_dev        (io_req_dev),
        .io_req_val        (io_req_val),
        .io_req_cont       (io_req_cont),
        .io_out_valid      (io_out_valid),
        .io_out_ready      (io_out_ready),
        .io_out_a          (io_out_a),
        .io_out_b          (io_out_b),
        .dbg_mmio_valid    (1'b0),
        .dbg_mmio_ready    (),
        .dbg_mmio_dev      (4'd0),
        .dbg_mmio_op       (4'd0),
        .dbg_mmio_wdata    (8'd0),
        .dbg_mmio_rdata    (),
        .cycle_cnt         (cycle_cnt),
        .ext_irq           (ext_irq),
        .status_led        (status_led),
        .io_busy           (io_busy),
        .dma_cfg_valid     (),
        .dma_cfg_op        (),
        .dma_cfg_data      (),
        .dma_cfg_rdata16   (16'd0),
        .dma_evt_desc_done (1'b0),
        .dma_evt_chain_done(1'b0),
        .ppi_task_dma_start(),
        .dma_sercom_tx_valid(1'b0),
        .dma_sercom_tx_data(8'd0),
        .dma_sercom_tx_ready(),
        .dma_sercom_rx_valid(),
        .dma_sercom_rx_data(),
        .dma_sercom_rx_ready(1'b0),
        .dma_pio_tx_valid  (1'b0),
        .dma_pio_tx_data   (16'd0),
        .dma_pio_tx_ready  (),
        .dma_pio_rx_valid  (),
        .dma_pio_rx_data   (),
        .dma_pio_rx_ready  (1'b0),
        .dma_pwm0_cc_we    (1'b0),
        .dma_pwm1_cc_we    (1'b0),
        .dma_pwm_cc_wdata  (16'd0),
        .dma_timer0_cnt    (),
        .dma_timer1_cnt    (),
        .dma_pace_pwm0     (),
        .dma_pace_pwm1     (),
        .dma_accel_wr_a    (1'b0),
        .dma_accel_wr_b    (1'b0),
        .dma_accel_wdata   (32'd0),
        .dma_accel_ready   (),
        .dma_accel_rdata   (),
        .ppi_inj_valid     (),
        .ppi_inj_ready     (1'b0),
        .ppi_inj_a         (),
        .ppi_inj_b         (),
        .pin_ui_1          (spi_miso),
        .pin_ui_3          (uart_rx),
        .pin_ui_7          (ext_irq),
        .pin_uio_5_in      (scl_bus),
        .pin_uio_6_in      (sda_bus),
        .pin_uo_3          (spi_sck),
        .pin_uo_4          (uart_tx),
        .pin_uo_5          (spi_mosi),
        .pin_uo_6          (spi_cs_n),
        .pin_uo_7          (pin_uo_7),
        .pin_uio_5_out     (pin_uio_5_out),
        .pin_uio_5_oe      (pin_uio_5_oe),
        .pin_uio_6_out     (pin_uio_6_out),
        .pin_uio_6_oe      (pin_uio_6_oe),
        .status_pin_sig    (status_led)
    );

endmodule
