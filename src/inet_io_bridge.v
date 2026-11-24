/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Interaction-Net IO Bridge & Nordic/RP2040 Peripheral Hub.
 *   - Hosts unified reconfigurable SERCOM (dev 0=UART, 1=SPI, 2=I2C), SYS (dev 3),
 *     2x Timer/PWM (dev 4), 4-ch PPI/DPPI (dev 5), EasyDMA reg port (dev 6),
 *     4xINT8/Q15 MAC Accelerator (dev 7), RP2040 PIO (dev 8), and GPIO Crossbar (dev 9).
 *   - Supports both non-blocking INet NUM >< IO continuations and SWD Region 0x7 MMIO.
 */

`default_nettype none

module inet_io_bridge (
    input  wire        clk,
    input  wire        rst_n,

    // INet IO interaction: NUM(io_req_val) >< IO(dev=io_req_dev, cont=io_req_cont)
    input  wire        io_req_valid,
    output wire        io_req_ready,
    input  wire [3:0]  io_req_dev,
    input  wire [15:0] io_req_val,   // [11:8] op, [7:0] data
    input  wire [23:0] io_req_cont,

    // Completed active pair: io_out_a ~ io_out_b
    output wire        io_out_valid,
    input  wire        io_out_ready,
    output wire [23:0] io_out_a,
    output wire [23:0] io_out_b,

    // SWD Region 0x7 MMIO port
    input  wire        dbg_mmio_valid,
    output reg         dbg_mmio_ready,
    input  wire [3:0]  dbg_mmio_dev,
    input  wire [3:0]  dbg_mmio_op,
    input  wire [7:0]  dbg_mmio_wdata,
    output wire [15:0] dbg_mmio_rdata,

    // System telemetry & status
    input  wire [31:0] cycle_cnt,
    input  wire        ext_irq,
    output reg         status_led,
    output wire        io_busy,

    // External DMA control & streaming ports (to inet_dma)
    output reg         dma_cfg_valid,
    output wire [3:0]  dma_cfg_op,
    output wire [7:0]  dma_cfg_data,
    input  wire [15:0] dma_cfg_rdata16,
    input  wire        dma_evt_desc_done,
    input  wire        dma_evt_chain_done,
    output wire        ppi_task_dma_start,

    input  wire        dma_sercom_tx_valid,
    input  wire [7:0]  dma_sercom_tx_data,
    output wire        dma_sercom_tx_ready,
    output wire        dma_sercom_rx_valid,
    output wire [7:0]  dma_sercom_rx_data,
    input  wire        dma_sercom_rx_ready,

    input  wire        dma_pio_tx_valid,
    input  wire [15:0] dma_pio_tx_data,
    output wire        dma_pio_tx_ready,
    output wire        dma_pio_rx_valid,
    output wire [15:0] dma_pio_rx_data,
    input  wire        dma_pio_rx_ready,

    input  wire        dma_pwm0_cc_we,
    input  wire        dma_pwm1_cc_we,
    input  wire [15:0] dma_pwm_cc_wdata,
    output wire [15:0] dma_timer0_cnt,
    output wire [15:0] dma_timer1_cnt,
    output wire        dma_pace_pwm0,
    output wire        dma_pace_pwm1,

    input  wire        dma_accel_wr_a,
    input  wire        dma_accel_wr_b,
    input  wire [31:0] dma_accel_wdata,
    output wire        dma_accel_ready,
    output wire [31:0] dma_accel_rdata,

    // Autonomous PPI active-pair injection (to inet_core)
    output wire        ppi_inj_valid,
    input  wire        ppi_inj_ready,
    output wire [23:0] ppi_inj_a,
    output wire [23:0] ppi_inj_b,

    // Non-QSPI/SWD physical pins (routed via inet_gpio_xbar)
    input  wire        pin_ui_1,
    input  wire        pin_ui_3,
    input  wire        pin_ui_7,
    input  wire        pin_uio_5_in,
    input  wire        pin_uio_6_in,
    output wire        pin_uo_3,
    output wire        pin_uo_4,
    output wire        pin_uo_5,
    output wire        pin_uo_6,
    output wire        pin_uo_7,
    output wire        pin_uio_5_out,
    output wire        pin_uio_5_oe,
    output wire        pin_uio_6_out,
    output wire        pin_uio_6_oe,
    input  wire        status_pin_sig
);

    localparam [3:0] TAG_NUM = 4'h3;

    /* verilator lint_off UNUSEDPARAM */
    localparam [3:0] DEV_UART  = 4'd0;
    localparam [3:0] DEV_SPI   = 4'd1;
    /* verilator lint_on UNUSEDPARAM */
    localparam [3:0] DEV_I2C   = 4'd2;
    localparam [3:0] DEV_SYS   = 4'd3;
    localparam [3:0] DEV_TIMER = 4'd4;
    localparam [3:0] DEV_PPI   = 4'd5;
    localparam [3:0] DEV_DMA   = 4'd6;
    localparam [3:0] DEV_ACCEL = 4'd7;
    localparam [3:0] DEV_PIO   = 4'd8;
    localparam [3:0] DEV_XBAR  = 4'd9;

    localparam [3:0] SYS_NOP        = 4'd0;
    localparam [3:0] SYS_CYCLE_LO   = 4'd1;
    localparam [3:0] SYS_CYCLE_HI   = 4'd2;
    localparam [3:0] SYS_GPIO_RD    = 4'd3;
    localparam [3:0] SYS_STATUS_LED = 4'd4;

    wire [3:0] req_op   = io_req_val[11:8];
    wire [7:0] req_data = io_req_val[7:0];
    wire _unused_val    = &{1'b0, io_req_val[15:12]};

    // ---------------- Submodule Handshakes & PPI Crossbar ----------------
    // SERCOM
    reg        sercom_req_valid;
    wire       sercom_req_ready;
    reg  [1:0] sercom_req_mode;
    reg        sercom_req_mode_auto;
    reg  [3:0] sercom_req_op;
    reg  [7:0] sercom_req_data;
    wire       sercom_resp_valid;
    wire       sercom_resp_ready = 1'b1;
    wire [7:0] sercom_resp_status;
    wire [7:0] sercom_resp_data;
    wire       sercom_evt_done;
    wire       sercom_evt_rx;
    wire [1:0] sercom_cur_mode;

    wire uart_rx_mux, uart_tx;
    wire spi_sck, spi_mosi, spi_miso_mux, spi_cs_n;
    wire i2c_scl_in_mux, i2c_scl_drive_low, i2c_sda_in_mux, i2c_sda_drive_low;

    // Shared op/data bus across synchronous/multi-cycle sub-peripherals
    reg  [3:0]  sub_op;
    reg  [7:0]  sub_data;
    assign dma_cfg_op   = sub_op;
    assign dma_cfg_data = sub_data;

    // Timer/PWM
    reg         timer_req_valid;
    wire [15:0] timer_resp_data16;
    wire        evt_t0_cmp, evt_t0_period, evt_t1_cmp, evt_t1_period;
    wire        pwm0_out, pwm1_out;
    wire        timer_cap_mux;

    // PPI
    reg         ppi_req_valid;
    wire [15:0] ppi_resp_data16;
    wire [7:0]  ppi_task_out;
    wire        ppi_gpio_mux;

    // Accel
    reg         accel_req_valid;
    wire        accel_resp_valid;
    wire [15:0] accel_resp_data16;
    wire        accel_evt_done;

    // PIO
    reg         pio_req_valid;
    wire [15:0] pio_resp_data16;
    wire        pio_evt_irq, pio_evt_rx_ne;
    wire [3:0]  pio_pins_in_mux, pio_pins_out, pio_pins_oe;

    // GPIO Crossbar
    reg         xbar_req_valid;
    wire [15:0] xbar_resp_data16;

    wire [1:0] _unused_sercom_mode = sercom_cur_mode;
    wire       _unused_ppi_task7   = ppi_task_out[7];

    inet_sercom u_sercom (
        .clk              (clk),
        .rst_n            (rst_n),
        .req_valid        (sercom_req_valid),
        .req_ready        (sercom_req_ready),
        .req_mode         (sercom_req_mode),
        .req_mode_auto    (sercom_req_mode_auto),
        .req_op           (sercom_req_op),
        .req_data         (sercom_req_data),
        .resp_valid       (sercom_resp_valid),
        .resp_ready       (sercom_resp_ready),
        .resp_status      (sercom_resp_status),
        .resp_data        (sercom_resp_data),
        .evt_done         (sercom_evt_done),
        .evt_rx           (sercom_evt_rx),
        .task_kick        (ppi_task_out[3]),
        .dma_tx_valid     (dma_sercom_tx_valid),
        .dma_tx_data      (dma_sercom_tx_data),
        .dma_tx_ready     (dma_sercom_tx_ready),
        .dma_rx_valid     (dma_sercom_rx_valid),
        .dma_rx_data      (dma_sercom_rx_data),
        .dma_rx_ready     (dma_sercom_rx_ready),
        .cur_mode         (sercom_cur_mode),
        .uart_rx          (uart_rx_mux),
        .uart_tx          (uart_tx),
        .spi_sck          (spi_sck),
        .spi_mosi         (spi_mosi),
        .spi_miso         (spi_miso_mux),
        .spi_cs_n         (spi_cs_n),
        .i2c_scl_in       (i2c_scl_in_mux),
        .i2c_scl_drive_low(i2c_scl_drive_low),
        .i2c_sda_in       (i2c_sda_in_mux),
        .i2c_sda_drive_low(i2c_sda_drive_low)
    );

    inet_timer u_timer (
        .clk              (clk),
        .rst_n            (rst_n),
        .req_valid        (timer_req_valid),
        .req_op           (sub_op),
        .req_data         (sub_data),
        .resp_data16      (timer_resp_data16),
        .t0_task_start_clr(ppi_task_out[0]),
        .t0_task_stop     (ppi_task_out[1]),
        .t1_task_count    (ppi_task_out[2]),
        .t1_task_start_clr(1'b0),
        .ext_cap_in       (timer_cap_mux),
        .evt_t0_cmp       (evt_t0_cmp),
        .evt_t0_period    (evt_t0_period),
        .evt_t1_cmp       (evt_t1_cmp),
        .evt_t1_period    (evt_t1_period),
        .pwm0_out         (pwm0_out),
        .pwm1_out         (pwm1_out),
        .dma_cc0_we       (dma_pwm0_cc_we),
        .dma_cc1_we       (dma_pwm1_cc_we),
        .dma_cc_wdata     (dma_pwm_cc_wdata),
        .dma_cnt0_rdata   (dma_timer0_cnt),
        .dma_cnt1_rdata   (dma_timer1_cnt)
    );

    assign dma_pace_pwm0 = evt_t0_period;
    assign dma_pace_pwm1 = evt_t1_period;
    assign ppi_task_dma_start = ppi_task_out[4];

    inet_ppi u_ppi (
        .clk          (clk),
        .rst_n        (rst_n),
        .req_valid    (ppi_req_valid),
        .req_op       (sub_op),
        .req_data     (sub_data),
        .resp_data16  (ppi_resp_data16),
        .evt_in_ext   ({pio_evt_rx_ne, pio_evt_irq, accel_evt_done,
                        dma_evt_chain_done, dma_evt_desc_done,
                        sercom_evt_rx, sercom_evt_done,
                        evt_t1_period, evt_t1_cmp, evt_t0_period, evt_t0_cmp}),
        .ppi_gpio_in  (ppi_gpio_mux),
        .task_out     (ppi_task_out),
        .ppi_inj_valid(ppi_inj_valid),
        .ppi_inj_ready(ppi_inj_ready),
        .ppi_inj_a    (ppi_inj_a),
        .ppi_inj_b    (ppi_inj_b)
    );

    inet_accel u_accel (
        .clk        (clk),
        .rst_n      (rst_n),
        .req_valid  (accel_req_valid),
        .req_op     (sub_op),
        .req_data   (sub_data),
        .resp_valid (accel_resp_valid),
        .resp_data16(accel_resp_data16),
        .dma_wr_a   (dma_accel_wr_a),
        .dma_wr_b   (dma_accel_wr_b),
        .dma_wdata  (dma_accel_wdata),
        .dma_ready  (dma_accel_ready),
        .dma_rdata  (dma_accel_rdata),
        .task_start (ppi_task_out[5]),
        .evt_done   (accel_evt_done)
    );

    inet_pio u_pio (
        .clk         (clk),
        .rst_n       (rst_n),
        .req_valid   (pio_req_valid),
        .req_op      (sub_op),
        .req_data    (sub_data),
        .resp_data16 (pio_resp_data16),
        .dma_tx_valid(dma_pio_tx_valid),
        .dma_tx_data (dma_pio_tx_data),
        .dma_tx_ready(dma_pio_tx_ready),
        .dma_rx_valid(dma_pio_rx_valid),
        .dma_rx_data (dma_pio_rx_data),
        .dma_rx_ready(dma_pio_rx_ready),
        .task_start  (ppi_task_out[6]),
        .evt_irq     (pio_evt_irq),
        .evt_rx_ne   (pio_evt_rx_ne),
        .pins_in     (pio_pins_in_mux),
        .pins_out    (pio_pins_out),
        .pins_oe     (pio_pins_oe)
    );

    inet_gpio_xbar u_xbar (
        .clk              (clk),
        .rst_n            (rst_n),
        .req_valid        (xbar_req_valid),
        .req_op           (sub_op),
        .req_data         (sub_data),
        .resp_data16      (xbar_resp_data16),
        .pin_ui_1         (pin_ui_1),
        .pin_ui_3         (pin_ui_3),
        .pin_ui_7         (pin_ui_7),
        .pin_uio_5_in     (pin_uio_5_in),
        .pin_uio_6_in     (pin_uio_6_in),
        .pin_uo_3         (pin_uo_3),
        .pin_uo_4         (pin_uo_4),
        .pin_uo_5         (pin_uo_5),
        .pin_uo_6         (pin_uo_6),
        .pin_uo_7         (pin_uo_7),
        .pin_uio_5_out    (pin_uio_5_out),
        .pin_uio_5_oe     (pin_uio_5_oe),
        .pin_uio_6_out    (pin_uio_6_out),
        .pin_uio_6_oe     (pin_uio_6_oe),
        .uart_tx          (uart_tx),
        .spi_sck          (spi_sck),
        .spi_mosi         (spi_mosi),
        .spi_cs_n         (spi_cs_n),
        .i2c_scl_drive_low(i2c_scl_drive_low),
        .i2c_sda_drive_low(i2c_sda_drive_low),
        .pwm0_out         (pwm0_out),
        .pwm1_out         (pwm1_out),
        .pio_pins_out     (pio_pins_out),
        .pio_pins_oe      (pio_pins_oe),
        .status_out       (status_pin_sig),
        .uart_rx_mux      (uart_rx_mux),
        .spi_miso_mux     (spi_miso_mux),
        .i2c_scl_in_mux   (i2c_scl_in_mux),
        .i2c_sda_in_mux   (i2c_sda_in_mux),
        .pio_pins_in_mux  (pio_pins_in_mux),
        .timer_cap_mux    (timer_cap_mux),
        .ppi_gpio_mux     (ppi_gpio_mux)
    );

    // ---------------- Unified 2-Slot SERCOM Queue + Fast/Sync Arbitration ----------------
    reg        sc_act_valid, sc_act_from_dbg;
    reg [23:0] sc_act_cont;

    reg        sc_q_valid;
    reg [1:0]  sc_q_mode;
    reg [3:0]  sc_q_op;
    reg [7:0]  sc_q_data;
    reg [23:0] sc_q_cont;

    reg        sercom_done_valid;
    reg [15:0] sercom_res;

    reg        fast_slot_busy, fast_done;
    reg [23:0] fast_cont;
    reg [15:0] fast_res;

    reg        dbg_active;
    reg [3:0]  dbg_active_dev;
    reg        fast_wait_sync;
    reg [3:0]  fast_wait_dev;
    reg        fast_is_dbg;

    assign dbg_mmio_rdata = fast_res;

    wire sercom_sel = (io_req_dev <= DEV_I2C);
    wire fast_sel   = !sercom_sel;

    assign io_req_ready = !dbg_mmio_valid && !dbg_active && (
        sercom_sel ? !sc_q_valid : (!fast_slot_busy && !fast_wait_sync)
    );

    reg        rr_bit;
    wire       sel_found = sercom_done_valid || fast_done;
    wire       sel_is_sc = sercom_done_valid && (!fast_done || !rr_bit);
    wire [23:0] sel_cont = sel_is_sc ? sc_act_cont : fast_cont;
    wire [15:0] sel_res16 = sel_is_sc ? sercom_res : fast_res;

    assign io_out_valid = sel_found;
    assign io_out_a     = sel_cont;
    assign io_out_b     = {TAG_NUM, 4'h0, sel_res16};

    assign io_busy = sc_act_valid | sc_q_valid | sercom_done_valid | fast_slot_busy | dbg_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sc_act_valid      <= 1'b0;
            sc_act_from_dbg   <= 1'b0;
            sc_act_cont       <= 24'd0;
            sc_q_valid        <= 1'b0;
            sc_q_mode         <= 2'd0;
            sc_q_op           <= 4'd0;
            sc_q_data         <= 8'd0;
            sc_q_cont         <= 24'd0;
            sercom_done_valid <= 1'b0;
            sercom_res        <= 16'd0;

            fast_slot_busy    <= 1'b0;
            fast_done         <= 1'b0;
            fast_cont         <= 24'd0;
            fast_res          <= 16'd0;

            sercom_req_valid     <= 1'b0;
            sercom_req_mode      <= 2'd0;
            sercom_req_mode_auto <= 1'b1;
            sercom_req_op        <= 4'd0;
            sercom_req_data      <= 8'd0;

            sub_op          <= 4'd0;
            sub_data        <= 8'd0;
            timer_req_valid <= 1'b0;
            ppi_req_valid   <= 1'b0;
            dma_cfg_valid   <= 1'b0;
            accel_req_valid <= 1'b0;
            pio_req_valid   <= 1'b0;
            xbar_req_valid  <= 1'b0;

            dbg_active     <= 1'b0;
            dbg_active_dev <= 4'd0;
            dbg_mmio_ready <= 1'b0;

            fast_wait_sync <= 1'b0;
            fast_wait_dev  <= 4'd0;
            fast_is_dbg    <= 1'b0;

            status_led     <= 1'b0;
            rr_bit         <= 1'b0;
        end else begin
            timer_req_valid <= 1'b0;
            ppi_req_valid   <= 1'b0;
            dma_cfg_valid   <= 1'b0;
            accel_req_valid <= 1'b0;
            pio_req_valid   <= 1'b0;
            xbar_req_valid  <= 1'b0;
            dbg_mmio_ready  <= 1'b0;

            if (sercom_req_valid && sercom_req_ready) begin
                sercom_req_valid <= 1'b0;
            end

            // Capture SERCOM response
            if (sc_act_valid && sercom_resp_valid) begin
                sc_act_valid <= 1'b0;
                if (sc_act_from_dbg) begin
                    dbg_active     <= 1'b0;
                    dbg_mmio_ready <= 1'b1;
                    fast_res       <= {sercom_resp_status, sercom_resp_data};
                end else begin
                    sercom_res        <= {sercom_resp_status, sercom_resp_data};
                    sercom_done_valid <= 1'b1;
                end
            end

            // Capture synchronous 1-cycle or Accel response
            if (fast_wait_sync) begin
                if (fast_wait_dev == DEV_ACCEL) begin
                    if (accel_resp_valid) begin
                        fast_wait_sync <= 1'b0;
                        fast_res       <= accel_resp_data16;
                        if (fast_is_dbg) begin
                            dbg_active     <= 1'b0;
                            dbg_mmio_ready <= 1'b1;
                        end else begin
                            fast_done <= 1'b1;
                        end
                    end
                end else if (fast_wait_dev == DEV_SYS) begin : sys_cap
                    reg [15:0] sys16;
                    case (sub_op)
                        SYS_NOP,
                        SYS_STATUS_LED: sys16 = {8'd0, sub_data};
                        SYS_CYCLE_LO:   sys16 = cycle_cnt[15:0];
                        SYS_CYCLE_HI:   sys16 = cycle_cnt[31:16];
                        SYS_GPIO_RD:    sys16 = {15'd0, ext_irq};
                        default:        sys16 = 16'hFF00;
                    endcase
                    fast_wait_sync <= 1'b0;
                    fast_res       <= sys16;
                    fast_done      <= 1'b1;
                end else begin : sync_cap
                    reg [15:0] r16;
                    case (fast_wait_dev)
                        DEV_TIMER: r16 = timer_resp_data16;
                        DEV_PPI:   r16 = ppi_resp_data16;
                        DEV_DMA:   r16 = dma_cfg_rdata16;
                        DEV_PIO:   r16 = pio_resp_data16;
                        DEV_XBAR:  r16 = xbar_resp_data16;
                        default:   r16 = 16'hFF00;
                    endcase
                    fast_wait_sync <= 1'b0;
                    fast_res       <= r16;
                    if (fast_is_dbg) begin
                        dbg_active     <= 1'b0;
                        dbg_mmio_ready <= 1'b1;
                    end else begin
                        fast_done <= 1'b1;
                    end
                end
            end

            // Dispatch pending SERCOM requests (when active slot & sercom_res are idle)
            if (!sc_act_valid && !sercom_req_valid && !sercom_done_valid) begin
                if (dbg_active && (dbg_active_dev <= DEV_I2C)) begin
                    sc_act_valid         <= 1'b1;
                    sc_act_from_dbg      <= 1'b1;
                    sercom_req_valid     <= 1'b1;
                    sercom_req_mode      <= dbg_active_dev[1:0];
                    sercom_req_mode_auto <= (dbg_mmio_op != 4'hF);
                    sercom_req_op        <= dbg_mmio_op;
                    sercom_req_data      <= dbg_mmio_wdata;
                end else if (sc_q_valid) begin
                    sc_q_valid           <= 1'b0;
                    sc_act_valid         <= 1'b1;
                    sc_act_from_dbg      <= 1'b0;
                    sc_act_cont          <= sc_q_cont;
                    sercom_req_valid     <= 1'b1;
                    sercom_req_mode      <= sc_q_mode;
                    sercom_req_mode_auto <= (sc_q_op != 4'hF);
                    sercom_req_op        <= sc_q_op;
                    sercom_req_data      <= sc_q_data;
                end
            end

            // Accept SWD Region 0x7 MMIO request
            if (dbg_mmio_valid && !dbg_active && !dbg_mmio_ready && !fast_wait_sync) begin
                if (dbg_mmio_dev <= DEV_I2C) begin
                    dbg_active     <= 1'b1;
                    dbg_active_dev <= dbg_mmio_dev;
                end else if (dbg_mmio_dev == DEV_SYS) begin
                    dbg_mmio_ready <= 1'b1;
                    case (dbg_mmio_op)
                        SYS_NOP:      fast_res <= {8'd0, dbg_mmio_wdata};
                        SYS_CYCLE_LO: fast_res <= cycle_cnt[15:0];
                        SYS_CYCLE_HI: fast_res <= cycle_cnt[31:16];
                        SYS_GPIO_RD:  fast_res <= {15'd0, ext_irq};
                        SYS_STATUS_LED: begin
                            status_led <= dbg_mmio_wdata[0];
                            fast_res   <= {8'd0, dbg_mmio_wdata};
                        end
                        default: fast_res <= 16'hFF00;
                    endcase
                end else if (dbg_mmio_dev <= DEV_XBAR) begin
                    dbg_active     <= 1'b1;
                    dbg_active_dev <= dbg_mmio_dev;
                    fast_wait_sync <= 1'b1;
                    fast_wait_dev  <= dbg_mmio_dev;
                    fast_is_dbg    <= 1'b1;
                    sub_op         <= dbg_mmio_op;
                    sub_data       <= dbg_mmio_wdata;
                    case (dbg_mmio_dev)
                        DEV_TIMER: timer_req_valid <= 1'b1;
                        DEV_PPI:   ppi_req_valid   <= 1'b1;
                        DEV_DMA:   dma_cfg_valid   <= 1'b1;
                        DEV_ACCEL: accel_req_valid <= 1'b1;
                        DEV_PIO:   pio_req_valid   <= 1'b1;
                        DEV_XBAR:  xbar_req_valid  <= 1'b1;
                        default: begin end
                    endcase
                end else begin
                    dbg_mmio_ready <= 1'b1;
                    fast_res       <= 16'hFF00;
                end
            end else if (io_req_valid && io_req_ready) begin
                // Accept INet NUM >< IO request
                if (sercom_sel) begin
                    if (!sc_act_valid && !sercom_req_valid && !sercom_done_valid && !sc_q_valid) begin
                        sc_act_valid         <= 1'b1;
                        sc_act_from_dbg      <= 1'b0;
                        sc_act_cont          <= io_req_cont;
                        sercom_req_valid     <= 1'b1;
                        sercom_req_mode      <= io_req_dev[1:0];
                        sercom_req_mode_auto <= (req_op != 4'hF);
                        sercom_req_op        <= req_op;
                        sercom_req_data      <= req_data;
                    end else begin
                        sc_q_valid <= 1'b1;
                        sc_q_mode  <= io_req_dev[1:0];
                        sc_q_op    <= req_op;
                        sc_q_data  <= req_data;
                        sc_q_cont  <= io_req_cont;
                    end
                end else if (fast_sel) begin
                    fast_slot_busy <= 1'b1;
                    fast_cont      <= io_req_cont;
                    if (io_req_dev <= DEV_XBAR) begin
                        if (io_req_dev == DEV_SYS && req_op == SYS_STATUS_LED) begin
                            status_led <= req_data[0];
                        end
                        fast_wait_sync <= 1'b1;
                        fast_wait_dev  <= io_req_dev;
                        fast_is_dbg    <= 1'b0;
                        sub_op         <= req_op;
                        sub_data       <= req_data;
                        case (io_req_dev)
                            DEV_TIMER: timer_req_valid <= 1'b1;
                            DEV_PPI:   ppi_req_valid   <= 1'b1;
                            DEV_DMA:   dma_cfg_valid   <= 1'b1;
                            DEV_ACCEL: accel_req_valid <= 1'b1;
                            DEV_PIO:   pio_req_valid   <= 1'b1;
                            DEV_XBAR:  xbar_req_valid  <= 1'b1;
                            default: begin end
                        endcase
                    end else begin
                        fast_done <= 1'b1;
                        fast_res  <= 16'hFF00;
                    end
                end
            end

            // Clear completed output once accepted
            if (sel_found && io_out_ready) begin
                rr_bit <= ~sel_is_sc;
                if (sel_is_sc) sercom_done_valid <= 1'b0;
                else begin
                    fast_slot_busy <= 1'b0;
                    fast_done      <= 1'b0;
                end
            end
        end
    end

endmodule
