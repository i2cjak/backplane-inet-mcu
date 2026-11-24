/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Top-level Tiny Tapeout SKY130 8x2 wrapper: TT_INet_MCU
 *   - Multi-tile Interaction-Net MCU with Latch-RF Hot Storage, QSPI Flash XIP + Dual PSRAM
 *   - Unified Reconfigurable SERCOM (UART/SPI/I2C), 2x Timer/PWM, 4-ch PPI/DPPI,
 *     Universal Linked-List EasyDMA, 4xINT8/Q15 MAC Accelerator, RP2040-style PIO,
 *     Nordic PSEL GPIO Crossbar (on non-QSPI/non-SWD pins), and ARM ADIv5 SWD Debug Target.
 */

`default_nettype none

module tt_um_inet_mcu #(
    parameter integer HOT_NODES       = 1,
    parameter integer HOT_WIRES       = 1,
    parameter integer REF_DIR_ENTRIES = 0,
    parameter integer N_TILES         = 2
) (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // Static / Hardwired QSPI & SWD Pins
    wire       qspi_pmod_mode = ui_in[6];
    wire       swclk_in       = qspi_pmod_mode ? 1'b0 : ui_in[0];
    wire       boot_halt      = ui_in[2];
    wire [1:0] qspi_lat       = ui_in[5:4];
    wire       ext_irq        = ui_in[7];

    // Core <-> QSPI Arbiter
    wire        core_qspi_cmd_valid, core_qspi_cmd_ready;
    wire [1:0]  core_qspi_cmd_dev;
    wire        core_qspi_cmd_write;
    wire [23:0] core_qspi_cmd_addr;
    wire [4:0]  core_qspi_cmd_words;
    wire [31:0] core_qspi_wdata;
    wire        core_qspi_wdata_valid, core_qspi_wdata_ready;

    // DMA <-> QSPI Arbiter
    wire        dma_qspi_cmd_valid, dma_qspi_cmd_ready;
    wire [1:0]  dma_qspi_cmd_dev;
    wire        dma_qspi_cmd_write;
    wire [23:0] dma_qspi_cmd_addr;
    wire [4:0]  dma_qspi_cmd_words;
    wire [31:0] dma_qspi_wdata;
    wire        dma_qspi_wdata_valid, dma_qspi_wdata_ready;

    // Shared QSPI controller
    wire        qspi_cmd_valid, qspi_cmd_ready;
    wire [1:0]  qspi_cmd_dev;
    wire        qspi_cmd_write;
    wire [23:0] qspi_cmd_addr;
    wire [4:0]  qspi_cmd_words;
    wire [31:0] qspi_wdata;
    wire        qspi_wdata_valid, qspi_wdata_ready;
    wire [31:0] qspi_rdata;
    wire        qspi_rdata_valid, qspi_rdata_ready;
    wire        qspi_busy;
    wire        dma_busy;

    reg         qspi_owner_dma; // 0=core, 1=dma (non-preemptive lock for active transaction)
    reg         qspi_arb_turn;  // 0=core turn, 1=dma turn when dma_busy
    wire        qspi_grant_dma = qspi_owner_dma || dma_qspi_cmd_valid;
    wire        qspi_idle_free = !qspi_busy && !core_qspi_cmd_valid && !dma_qspi_cmd_valid && qspi_cmd_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            qspi_owner_dma <= 1'b0;
            qspi_arb_turn  <= 1'b0;
        end else begin
            if (!qspi_busy && qspi_cmd_valid && qspi_cmd_ready) begin
                qspi_owner_dma <= qspi_grant_dma;
            end else if (!qspi_busy && !qspi_cmd_valid) begin
                qspi_owner_dma <= 1'b0;
                if (dma_busy) qspi_arb_turn <= ~qspi_arb_turn;
                else          qspi_arb_turn <= 1'b0;
            end
        end
    end

    assign qspi_cmd_valid        = qspi_grant_dma ? dma_qspi_cmd_valid  : core_qspi_cmd_valid;
    assign qspi_cmd_dev          = qspi_grant_dma ? dma_qspi_cmd_dev    : core_qspi_cmd_dev;
    assign qspi_cmd_write        = qspi_grant_dma ? dma_qspi_cmd_write  : core_qspi_cmd_write;
    assign qspi_cmd_addr         = qspi_grant_dma ? dma_qspi_cmd_addr   : core_qspi_cmd_addr;
    assign qspi_cmd_words        = qspi_grant_dma ? dma_qspi_cmd_words  : core_qspi_cmd_words;
    assign qspi_wdata            = qspi_grant_dma ? dma_qspi_wdata      : core_qspi_wdata;
    assign qspi_wdata_valid      = qspi_grant_dma ? dma_qspi_wdata_valid: core_qspi_wdata_valid;
    assign qspi_rdata_ready      = 1'b1;

    assign core_qspi_cmd_ready   = !qspi_grant_dma && qspi_idle_free && !(dma_busy && qspi_arb_turn);
    assign core_qspi_wdata_ready = !qspi_owner_dma && qspi_wdata_ready;
    wire   core_qspi_rdata_valid = !qspi_owner_dma && qspi_rdata_valid;
    wire   core_qspi_busy        = qspi_busy || (qspi_owner_dma && qspi_cmd_valid);

    assign dma_qspi_cmd_ready    = qspi_idle_free && qspi_arb_turn;
    assign dma_qspi_wdata_ready  = qspi_owner_dma && qspi_wdata_ready;
    wire   dma_qspi_rdata_valid  = qspi_owner_dma && qspi_rdata_valid;
    wire   dma_qspi_busy         = qspi_busy || (!qspi_owner_dma && qspi_cmd_valid);
    wire   core_qspi_rdata_ready, dma_qspi_rdata_ready;

    wire        qspi_cs0_n, qspi_cs1_n, qspi_cs2_n, qspi_sck;
    wire [3:0]  qspi_io_out, qspi_io_oe, qspi_io_in;

    // IO Bridge, DMA, PPI, Accel, PIO, Crossbar interconnects
    wire        io_req_valid, io_req_ready;
    wire [3:0]  io_req_dev;
    wire [15:0] io_req_val;
    wire [23:0] io_req_cont;
    wire        io_out_valid, io_out_ready;
    wire [23:0] io_out_a, io_out_b;
    wire        io_busy, status_led;

    wire        dbg_mmio_valid, dbg_mmio_ready;
    wire [3:0]  dbg_mmio_dev, dbg_mmio_op;
    wire [7:0]  dbg_mmio_wdata;
    wire [15:0] dbg_mmio_rdata;

    wire        dma_cfg_valid;
    wire [3:0]  dma_cfg_op;
    wire [7:0]  dma_cfg_data;
    wire [15:0] dma_cfg_rdata16;
    wire        dma_evt_desc_done, dma_evt_chain_done;
    wire        ppi_task_dma_start;

    wire        dma_sercom_tx_valid, dma_sercom_tx_ready;
    wire [7:0]  dma_sercom_tx_data;
    wire        dma_sercom_rx_valid, dma_sercom_rx_ready;
    wire [7:0]  dma_sercom_rx_data;

    wire        dma_pio_tx_valid, dma_pio_tx_ready;
    wire [15:0] dma_pio_tx_data;
    wire        dma_pio_rx_valid, dma_pio_rx_ready;
    wire [15:0] dma_pio_rx_data;

    wire        dma_pwm0_cc_we, dma_pwm1_cc_we;
    wire [15:0] dma_pwm_cc_wdata, dma_timer0_cnt, dma_timer1_cnt;
    wire        dma_pace_pwm0, dma_pace_pwm1;

    wire        dma_accel_wr_a, dma_accel_wr_b, dma_accel_ready;
    wire [31:0] dma_accel_wdata, dma_accel_rdata;

    wire        ppi_inj_valid, ppi_inj_ready;
    wire [23:0] ppi_inj_a, ppi_inj_b;
    wire        dma_inj_valid, dma_inj_ready;
    wire [23:0] dma_inj_a, dma_inj_b;

    wire        core_inj_valid = ppi_inj_valid | dma_inj_valid;
    wire        core_inj_ready;
    wire [23:0] core_inj_a = ppi_inj_valid ? ppi_inj_a : dma_inj_a;
    wire [23:0] core_inj_b = ppi_inj_valid ? ppi_inj_b : dma_inj_b;
    assign      ppi_inj_ready = core_inj_ready && ppi_inj_valid;
    assign      dma_inj_ready = core_inj_ready && !ppi_inj_valid && dma_inj_valid;

    // Crossbar non-QSPI/SWD physical pin wires
    wire xbar_uo_3, xbar_uo_4, xbar_uo_5, xbar_uo_6, xbar_uo_7;
    wire xbar_uio_5_out, xbar_uio_5_oe, xbar_uio_6_out, xbar_uio_6_oe;

    // SWD Debug & Telemetry
    wire        swdio_in, swdio_out, swdio_oe;
    wire        dbg_halt_req, dbg_step_pulse, dbg_reset_pulse;
    wire        core_halted, core_busy;
    wire [3:0]  active_tiles;
    wire [31:0] cycle_cnt, redex_cnt, qspi_txn_cnt, queue_depths;
    wire        dbg_mem_req, dbg_mem_we;
    wire [23:0] dbg_mem_addr;
    wire [31:0] dbg_mem_wdata, dbg_mem_rdata;
    wire        dbg_mem_ready;

    // Static QSPI/SWD + Crossbar Pin Mux
    assign qspi_io_in = qspi_pmod_mode
        ? {uio_in[5], uio_in[4], uio_in[2], uio_in[1]}
        : uio_in[3:0];

    assign swdio_in = qspi_pmod_mode ? 1'b1 : uio_in[4];

    assign uio_out = qspi_pmod_mode
        ? {qspi_cs2_n, qspi_cs1_n, qspi_io_out[3], qspi_io_out[2],
           qspi_sck, qspi_io_out[1], qspi_io_out[0], qspi_cs0_n}
        : {qspi_cs2_n, xbar_uio_6_out, xbar_uio_5_out, swdio_out, qspi_io_out};

    assign uio_oe = qspi_pmod_mode
        ? {1'b1, 1'b1, qspi_io_oe[3], qspi_io_oe[2],
           1'b1, qspi_io_oe[1], qspi_io_oe[0], 1'b1}
        : {1'b1, xbar_uio_6_oe, xbar_uio_5_oe, swdio_oe, qspi_io_oe};

    assign uo_out = {
        xbar_uo_7,
        xbar_uo_6,
        xbar_uo_5,
        xbar_uo_4,
        xbar_uo_3,
        qspi_sck,
        qspi_cs1_n,
        qspi_cs0_n
    };

    // ------------------------------------------------------------------------
    // Submodules
    // ------------------------------------------------------------------------
    inet_core #(
        .HOT_NODES      (HOT_NODES),
        .HOT_WIRES      (HOT_WIRES),
        .REF_DIR_ENTRIES(REF_DIR_ENTRIES),
        .N_TILES        (N_TILES)
    ) u_core (
        .clk             (clk),
        .rst_n           (rst_n),
        .qspi_cmd_valid  (core_qspi_cmd_valid),
        .qspi_cmd_ready  (core_qspi_cmd_ready),
        .qspi_cmd_dev    (core_qspi_cmd_dev),
        .qspi_cmd_write  (core_qspi_cmd_write),
        .qspi_cmd_addr   (core_qspi_cmd_addr),
        .qspi_cmd_words  (core_qspi_cmd_words),
        .qspi_wdata      (core_qspi_wdata),
        .qspi_wdata_valid(core_qspi_wdata_valid),
        .qspi_wdata_ready(core_qspi_wdata_ready),
        .qspi_rdata      (qspi_rdata),
        .qspi_rdata_valid(core_qspi_rdata_valid),
        .qspi_rdata_ready(core_qspi_rdata_ready),
        .qspi_busy       (core_qspi_busy),
        .io_req_valid    (io_req_valid),
        .io_req_ready    (io_req_ready),
        .io_req_dev      (io_req_dev),
        .io_req_val      (io_req_val),
        .io_req_cont     (io_req_cont),
        .io_out_valid    (io_out_valid),
        .io_out_ready    (io_out_ready),
        .io_out_a        (io_out_a),
        .io_out_b        (io_out_b),
        .io_busy         (io_busy || dma_busy),
        .ppi_inj_valid   (core_inj_valid),
        .ppi_inj_ready   (core_inj_ready),
        .ppi_inj_a       (core_inj_a),
        .ppi_inj_b       (core_inj_b),
        .dbg_mmio_valid  (dbg_mmio_valid),
        .dbg_mmio_ready  (dbg_mmio_ready),
        .dbg_mmio_dev    (dbg_mmio_dev),
        .dbg_mmio_op     (dbg_mmio_op),
        .dbg_mmio_wdata  (dbg_mmio_wdata),
        .dbg_mmio_rdata  (dbg_mmio_rdata),
        .boot_halt       (boot_halt),
        .dbg_halt_req    (dbg_halt_req),
        .dbg_step_pulse  (dbg_step_pulse),
        .dbg_reset_pulse (dbg_reset_pulse),
        .core_halted     (core_halted),
        .core_busy       (core_busy),
        .active_tiles    (active_tiles),
        .cycle_cnt       (cycle_cnt),
        .redex_cnt       (redex_cnt),
        .qspi_txn_cnt    (qspi_txn_cnt),
        .queue_depths    (queue_depths),
        .dbg_mem_req     (dbg_mem_req),
        .dbg_mem_we      (dbg_mem_we),
        .dbg_mem_addr    (dbg_mem_addr),
        .dbg_mem_wdata   (dbg_mem_wdata),
        .dbg_mem_ready   (dbg_mem_ready),
        .dbg_mem_rdata   (dbg_mem_rdata)
    );

    inet_qspi_ctrl u_qspi (
        .clk        (clk),
        .rst_n      (rst_n),
        .qspi_lat   (qspi_lat),
        .cmd_valid  (qspi_cmd_valid),
        .cmd_ready  (qspi_cmd_ready),
        .cmd_dev    (qspi_cmd_dev),
        .cmd_write  (qspi_cmd_write),
        .cmd_addr   (qspi_cmd_addr),
        .cmd_words  (qspi_cmd_words),
        .wdata      (qspi_wdata),
        .wdata_valid(qspi_wdata_valid),
        .wdata_ready(qspi_wdata_ready),
        .rdata      (qspi_rdata),
        .rdata_valid(qspi_rdata_valid),
        .rdata_ready(qspi_rdata_ready),
        .busy       (qspi_busy),
        .qspi_cs0_n (qspi_cs0_n),
        .qspi_cs1_n (qspi_cs1_n),
        .qspi_cs2_n (qspi_cs2_n),
        .qspi_sck   (qspi_sck),
        .qspi_io_out(qspi_io_out),
        .qspi_io_oe (qspi_io_oe),
        .qspi_io_in (qspi_io_in)
    );

    inet_io_bridge u_io (
        .clk                (clk),
        .rst_n              (rst_n),
        .io_req_valid       (io_req_valid),
        .io_req_ready       (io_req_ready),
        .io_req_dev         (io_req_dev),
        .io_req_val         (io_req_val),
        .io_req_cont        (io_req_cont),
        .io_out_valid       (io_out_valid),
        .io_out_ready       (io_out_ready),
        .io_out_a           (io_out_a),
        .io_out_b           (io_out_b),
        .dbg_mmio_valid     (dbg_mmio_valid),
        .dbg_mmio_ready     (dbg_mmio_ready),
        .dbg_mmio_dev       (dbg_mmio_dev),
        .dbg_mmio_op        (dbg_mmio_op),
        .dbg_mmio_wdata     (dbg_mmio_wdata),
        .dbg_mmio_rdata     (dbg_mmio_rdata),
        .cycle_cnt          (cycle_cnt),
        .ext_irq            (ext_irq),
        .status_led         (status_led),
        .io_busy            (io_busy),
        .dma_cfg_valid      (dma_cfg_valid),
        .dma_cfg_op         (dma_cfg_op),
        .dma_cfg_data       (dma_cfg_data),
        .dma_cfg_rdata16    (dma_cfg_rdata16),
        .dma_evt_desc_done  (dma_evt_desc_done),
        .dma_evt_chain_done (dma_evt_chain_done),
        .ppi_task_dma_start (ppi_task_dma_start),
        .dma_sercom_tx_valid(dma_sercom_tx_valid),
        .dma_sercom_tx_data (dma_sercom_tx_data),
        .dma_sercom_tx_ready(dma_sercom_tx_ready),
        .dma_sercom_rx_valid(dma_sercom_rx_valid),
        .dma_sercom_rx_data (dma_sercom_rx_data),
        .dma_sercom_rx_ready(dma_sercom_rx_ready),
        .dma_pio_tx_valid   (dma_pio_tx_valid),
        .dma_pio_tx_data    (dma_pio_tx_data),
        .dma_pio_tx_ready   (dma_pio_tx_ready),
        .dma_pio_rx_valid   (dma_pio_rx_valid),
        .dma_pio_rx_data    (dma_pio_rx_data),
        .dma_pio_rx_ready   (dma_pio_rx_ready),
        .dma_pwm0_cc_we     (dma_pwm0_cc_we),
        .dma_pwm1_cc_we     (dma_pwm1_cc_we),
        .dma_pwm_cc_wdata   (dma_pwm_cc_wdata),
        .dma_timer0_cnt     (dma_timer0_cnt),
        .dma_timer1_cnt     (dma_timer1_cnt),
        .dma_pace_pwm0      (dma_pace_pwm0),
        .dma_pace_pwm1      (dma_pace_pwm1),
        .dma_accel_wr_a     (dma_accel_wr_a),
        .dma_accel_wr_b     (dma_accel_wr_b),
        .dma_accel_wdata    (dma_accel_wdata),
        .dma_accel_ready    (dma_accel_ready),
        .dma_accel_rdata    (dma_accel_rdata),
        .ppi_inj_valid      (ppi_inj_valid),
        .ppi_inj_ready      (ppi_inj_ready),
        .ppi_inj_a          (ppi_inj_a),
        .ppi_inj_b          (ppi_inj_b),
        .pin_ui_1           (ui_in[1]),
        .pin_ui_3           (ui_in[3]),
        .pin_ui_7           (ext_irq),
        .pin_uio_5_in       (qspi_pmod_mode ? 1'b1 : uio_in[5]),
        .pin_uio_6_in       (qspi_pmod_mode ? 1'b1 : uio_in[6]),
        .pin_uo_3           (xbar_uo_3),
        .pin_uo_4           (xbar_uo_4),
        .pin_uo_5           (xbar_uo_5),
        .pin_uo_6           (xbar_uo_6),
        .pin_uo_7           (xbar_uo_7),
        .pin_uio_5_out      (xbar_uio_5_out),
        .pin_uio_5_oe       (xbar_uio_5_oe),
        .pin_uio_6_out      (xbar_uio_6_out),
        .pin_uio_6_oe       (xbar_uio_6_oe),
        .status_pin_sig     (core_busy | status_led)
    );

    inet_dma u_dma (
        .clk             (clk),
        .rst_n           (rst_n),
        .req_valid       (dma_cfg_valid),
        .req_op          (dma_cfg_op),
        .req_data        (dma_cfg_data),
        .resp_data16     (dma_cfg_rdata16),
        .qspi_cmd_valid  (dma_qspi_cmd_valid),
        .qspi_cmd_ready  (dma_qspi_cmd_ready),
        .qspi_cmd_dev    (dma_qspi_cmd_dev),
        .qspi_cmd_write  (dma_qspi_cmd_write),
        .qspi_cmd_addr   (dma_qspi_cmd_addr),
        .qspi_cmd_words  (dma_qspi_cmd_words),
        .qspi_wdata      (dma_qspi_wdata),
        .qspi_wdata_valid(dma_qspi_wdata_valid),
        .qspi_wdata_ready(dma_qspi_wdata_ready),
        .qspi_rdata      (qspi_rdata),
        .qspi_rdata_valid(dma_qspi_rdata_valid),
        .qspi_rdata_ready(dma_qspi_rdata_ready),
        .qspi_busy       (dma_qspi_busy),
        .sercom_tx_valid (dma_sercom_tx_valid),
        .sercom_tx_data  (dma_sercom_tx_data),
        .sercom_tx_ready (dma_sercom_tx_ready),
        .sercom_rx_valid (dma_sercom_rx_valid),
        .sercom_rx_data  (dma_sercom_rx_data),
        .sercom_rx_ready (dma_sercom_rx_ready),
        .pio_tx_valid    (dma_pio_tx_valid),
        .pio_tx_data     (dma_pio_tx_data),
        .pio_tx_ready    (dma_pio_tx_ready),
        .pio_rx_valid    (dma_pio_rx_valid),
        .pio_rx_data     (dma_pio_rx_data),
        .pio_rx_ready    (dma_pio_rx_ready),
        .pwm0_cc_we      (dma_pwm0_cc_we),
        .pwm1_cc_we      (dma_pwm1_cc_we),
        .pwm_cc_wdata    (dma_pwm_cc_wdata),
        .timer0_cnt      (dma_timer0_cnt),
        .timer1_cnt      (dma_timer1_cnt),
        .accel_wr_a      (dma_accel_wr_a),
        .accel_wr_b      (dma_accel_wr_b),
        .accel_wdata     (dma_accel_wdata),
        .accel_ready     (dma_accel_ready),
        .accel_rdata     (dma_accel_rdata),
        .dma_inj_valid   (dma_inj_valid),
        .dma_inj_ready   (dma_inj_ready),
        .dma_inj_a       (dma_inj_a),
        .dma_inj_b       (dma_inj_b),
        .task_start      (ppi_task_dma_start),
        .pace_pwm0       (dma_pace_pwm0),
        .pace_pwm1       (dma_pace_pwm1),
        .evt_desc_done   (dma_evt_desc_done),
        .evt_chain_done  (dma_evt_chain_done),
        .dma_busy        (dma_busy)
    );

    inet_swd u_swd (
        .clk            (clk),
        .rst_n          (rst_n),
        .swclk_in       (swclk_in),
        .swdio_in       (swdio_in),
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

    wire        _unused = &{ena, uio_in[7], uio_in[6], uio_in[3], uio_in[0],
                            core_qspi_rdata_ready, dma_qspi_rdata_ready, 1'b0};

endmodule
