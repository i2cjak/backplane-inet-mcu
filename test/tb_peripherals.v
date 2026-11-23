`default_nettype none
`timescale 1ns / 1ps

module tb_peripherals ();
    reg clk;
    reg rst_n;

    // Unified reconfigurable SERCOM
    reg        req_valid;
    wire       req_ready;
    reg  [1:0] req_mode;
    reg        req_mode_auto;
    reg  [3:0] req_op;
    reg  [7:0] req_data;
    wire       resp_valid;
    reg        resp_ready;
    wire [7:0] resp_status;
    wire [7:0] resp_data;
    wire       evt_done;
    wire       evt_rx;
    wire [1:0] cur_mode;

    reg        uart_rx;
    wire       uart_tx;

    wire       spi_sck;
    wire       spi_mosi;
    reg        spi_miso;
    wire       spi_cs_n;

    reg        ext_scl_low;
    reg        ext_sda_low;
    wire       scl_drive_low;
    wire       sda_drive_low;

    wire scl_bus = !(scl_drive_low | ext_scl_low);
    wire sda_bus = !(sda_drive_low | ext_sda_low);

    inet_sercom u_sercom (
        .clk              (clk),
        .rst_n            (rst_n),
        .req_valid        (req_valid),
        .req_ready        (req_ready),
        .req_mode         (req_mode),
        .req_mode_auto    (req_mode_auto),
        .req_op           (req_op),
        .req_data         (req_data),
        .resp_valid       (resp_valid),
        .resp_ready       (resp_ready),
        .resp_status      (resp_status),
        .resp_data        (resp_data),
        .evt_done         (evt_done),
        .evt_rx           (evt_rx),
        .task_kick        (1'b0),
        .dma_tx_valid     (1'b0),
        .dma_tx_data      (8'd0),
        .dma_tx_ready     (),
        .dma_rx_valid     (),
        .dma_rx_data      (),
        .dma_rx_ready     (1'b0),
        .cur_mode         (cur_mode),
        .uart_rx          (uart_rx),
        .uart_tx          (uart_tx),
        .spi_sck          (spi_sck),
        .spi_mosi         (spi_mosi),
        .spi_miso         (spi_miso),
        .spi_cs_n         (spi_cs_n),
        .i2c_scl_in       (scl_bus),
        .i2c_scl_drive_low(scl_drive_low),
        .i2c_sda_in       (sda_bus),
        .i2c_sda_drive_low(sda_drive_low)
    );

endmodule
