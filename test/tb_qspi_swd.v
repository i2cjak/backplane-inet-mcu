`default_nettype none
`timescale 1ns / 1ps

module tb_qspi_swd ();
    reg clk;
    reg rst_n;

    // QSPI controller
    reg  [1:0]  qspi_lat;
    reg         cmd_valid;
    wire        cmd_ready;
    reg  [1:0]  cmd_dev;
    reg         cmd_write;
    reg  [23:0] cmd_addr;
    reg  [4:0]  cmd_words;
    reg  [31:0] wdata;
    reg         wdata_valid;
    wire        wdata_ready;
    wire [31:0] rdata;
    wire        rdata_valid;
    reg         rdata_ready;
    wire        qspi_busy;
    wire        qspi_cs0_n;
    wire        qspi_cs1_n;
    wire        qspi_cs2_n;
    wire        qspi_sck;
    wire [3:0]  qspi_io_out;
    wire [3:0]  qspi_io_oe;
    reg  [3:0]  qspi_io_in;

    // SWD target
    reg         swclk_in;
    reg         host_swdio_in;
    wire        swdio_out;
    wire        swdio_oe;
    wire        swdio_bus = swdio_oe ? swdio_out : host_swdio_in;
    reg         boot_halt;
    wire        dbg_halt_req;
    wire        dbg_step_pulse;
    wire        dbg_reset_pulse;
    reg         core_halted;
    reg         core_busy;
    reg  [3:0]  active_tiles;
    reg  [31:0] cycle_cnt;
    reg  [31:0] redex_cnt;
    reg  [31:0] qspi_txn_cnt;
    reg  [31:0] queue_depths;
    wire        dbg_mem_req;
    wire        dbg_mem_we;
    wire [23:0] dbg_mem_addr;
    wire [31:0] dbg_mem_wdata;
    reg         dbg_mem_ready;
    reg  [31:0] dbg_mem_rdata;

    inet_qspi_ctrl u_qspi (
        .clk        (clk),
        .rst_n      (rst_n),
        .qspi_lat   (qspi_lat),
        .cmd_valid  (cmd_valid),
        .cmd_ready  (cmd_ready),
        .cmd_dev    (cmd_dev),
        .cmd_write  (cmd_write),
        .cmd_addr   (cmd_addr),
        .cmd_words  (cmd_words),
        .wdata      (wdata),
        .wdata_valid(wdata_valid),
        .wdata_ready(wdata_ready),
        .rdata      (rdata),
        .rdata_valid(rdata_valid),
        .rdata_ready(rdata_ready),
        .busy       (qspi_busy),
        .qspi_cs0_n (qspi_cs0_n),
        .qspi_cs1_n (qspi_cs1_n),
        .qspi_cs2_n (qspi_cs2_n),
        .qspi_sck   (qspi_sck),
        .qspi_io_out(qspi_io_out),
        .qspi_io_oe (qspi_io_oe),
        .qspi_io_in (qspi_io_in)
    );

    inet_swd u_swd (
        .clk            (clk),
        .rst_n          (rst_n),
        .swclk_in       (swclk_in),
        .swdio_in       (swdio_bus),
        .swdio_out      (swdio_out),
        .swdio_oe       (swdio_oe),
        .boot_halt      (boot_halt),
        .dbg_halt_req   (dbg_halt_req),
        .dbg_step_pulse (dbg_step_pulse),
        .dbg_reset_pulse(dbg_reset_pulse),
        .core_halted    (core_halted),
        .core_busy      (core_busy),
        .qspi_busy      (qspi_busy),
        .active_tiles   (active_tiles),
        .cycle_cnt      (cycle_cnt),
        .redex_cnt      (redex_cnt),
        .qspi_txn_cnt   (qspi_txn_cnt),
        .queue_depths   (queue_depths),
        .dbg_mem_req    (dbg_mem_req),
        .dbg_mem_we     (dbg_mem_we),
        .dbg_mem_addr   (dbg_mem_addr),
        .dbg_mem_wdata  (dbg_mem_wdata),
        .dbg_mem_ready  (dbg_mem_ready),
        .dbg_mem_rdata  (dbg_mem_rdata)
    );

endmodule
