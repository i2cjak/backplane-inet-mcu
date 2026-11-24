/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Universal Linked-List EasyDMA Engine (Nordic EasyDMA style).
 *   - Reads 4-word (16-byte) descriptors from PSRAM A (CS1_n):
 *       +0: src_addr[23:0]
 *       +4: dst_addr[23:0]
 *       +8: {flags[15:0], len[15:0]}
 *       +C: next_ptr[23:0] (0 = end of chain)
 *   - Streams seamlessly across PSRAM A, SERCOM TX/RX, PIO TX/RX, PWM0/PWM1 CC
 *     (with optional period-reload pacing), 4xINT8/Q15 Accelerator, and INet pairs.
 */

`default_nettype none

module inet_dma (
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (DEV_DMA = 6)
    input  wire        req_valid,
    input  wire [3:0]  req_op,
    input  wire [7:0]  req_data,
    output reg  [15:0] resp_data16,

    // Shared QSPI master port (PSRAM A: dev=2'd1)
    output reg         qspi_cmd_valid,
    input  wire        qspi_cmd_ready,
    output wire [1:0]  qspi_cmd_dev,
    output reg         qspi_cmd_write,
    output reg  [23:0] qspi_cmd_addr,
    output wire [4:0]  qspi_cmd_words,
    output wire [31:0] qspi_wdata,
    output reg         qspi_wdata_valid,
    input  wire        qspi_wdata_ready,
    input  wire [31:0] qspi_rdata,
    input  wire        qspi_rdata_valid,
    output wire        qspi_rdata_ready,
    input  wire        qspi_busy,

    // Peripheral DMA endpoints
    output reg         sercom_tx_valid,
    output wire [7:0]  sercom_tx_data,
    input  wire        sercom_tx_ready,
    input  wire        sercom_rx_valid,
    input  wire [7:0]  sercom_rx_data,
    output reg         sercom_rx_ready,

    output reg         pio_tx_valid,
    output wire [15:0] pio_tx_data,
    input  wire        pio_tx_ready,
    input  wire        pio_rx_valid,
    input  wire [15:0] pio_rx_data,
    output reg         pio_rx_ready,

    output reg         pwm0_cc_we,
    output reg         pwm1_cc_we,
    output wire [15:0] pwm_cc_wdata,
    input  wire [15:0] timer0_cnt,
    input  wire [15:0] timer1_cnt,

    output reg         accel_wr_a,
    output reg         accel_wr_b,
    output wire [31:0] accel_wdata,
    input  wire        accel_ready,
    input  wire [31:0] accel_rdata,

    output reg         dma_inj_valid,
    input  wire        dma_inj_ready,
    output reg  [23:0] dma_inj_a,
    output wire [23:0] dma_inj_b,

    // PPI tasks & events
    input  wire        task_start,
    input  wire        pace_pwm0,
    input  wire        pace_pwm1,
    output reg         evt_desc_done,
    output reg         evt_chain_done,
    output wire        dma_busy
);

    localparam [3:0] OP_DESC_LO    = 4'd0;
    localparam [3:0] OP_DESC_MID   = 4'd1;
    localparam [3:0] OP_DESC_HI    = 4'd2;
    localparam [3:0] OP_START      = 4'd3;
    localparam [3:0] OP_ABORT      = 4'd4;
    localparam [3:0] OP_STATUS     = 4'd5;
    localparam [3:0] OP_DESC_COUNT = 4'd6;

    localparam [2:0] SRC_PSRAM   = 3'd0;
    localparam [2:0] SRC_SERCOM  = 3'd1;
    localparam [2:0] SRC_PIO     = 3'd2;
    localparam [2:0] SRC_ACCEL   = 3'd3;
    localparam [2:0] SRC_TIMER0  = 3'd4;
    localparam [2:0] SRC_TIMER1  = 3'd5;
    localparam [2:0] SRC_ZERO    = 3'd6;

    localparam [2:0] DST_PSRAM     = 3'd0;
    localparam [2:0] DST_SERCOM    = 3'd1;
    localparam [2:0] DST_PIO       = 3'd2;
    localparam [2:0] DST_PWM0      = 3'd3;
    localparam [2:0] DST_PWM1      = 3'd4;
    localparam [2:0] DST_ACCEL_AB  = 3'd5;
    localparam [2:0] DST_ACCEL_ONE = 3'd6;
    localparam [2:0] DST_INET_PAIR = 3'd7;

    localparam [2:0] D_IDLE     = 3'd0;
    localparam [2:0] D_FETCH_D  = 3'd1;
    localparam [2:0] D_PACE     = 3'd2;
    localparam [2:0] D_READ     = 3'd3;
    localparam [2:0] D_WRITE    = 3'd4;
    localparam [2:0] D_INJ_WAIT = 3'd5;
    localparam [2:0] D_NEXT     = 3'd6;

    reg [2:0]  state;
    reg [13:0] desc_ptr, src_addr, dst_addr, next_ptr;
    reg [7:0]  rem_len;
    reg [11:0] flags;
    reg [31:0] data_buf;
    reg [1:0]  fetch_idx;
    reg        q_active;
    reg        parity_idx;
    reg        pace_armed;
    reg [5:0]  desc_count;
    reg        done_sticky;

    wire [2:0] src_sel   = flags[2:0];
    wire [2:0] dst_sel   = flags[5:3];
    wire       src_inc   = flags[6];
    wire       dst_inc   = flags[7];
    wire       chain_en  = flags[9];
    wire       pace_en0  = flags[10];
    wire       pace_en1  = flags[11];
    wire       _unused_flags = flags[8];

    assign qspi_cmd_dev     = 2'd1; // PSRAM A
    assign qspi_cmd_words   = (state == D_FETCH_D) ? 5'd3 : 5'd0;
    assign qspi_rdata_ready = 1'b1;
    assign dma_busy         = (state != D_IDLE);

    // Direct wire-driven outputs from single data_buf & state-muxed QSPI address
    assign qspi_wdata     = data_buf;
    assign sercom_tx_data = data_buf[7:0];
    assign pio_tx_data    = data_buf[15:0];
    assign pwm_cc_wdata   = data_buf[15:0];
    assign accel_wdata    = data_buf;
    assign dma_inj_b      = data_buf[23:0];

    wire [13:0] addr_plus1 = ((state == D_READ) ? src_addr : dst_addr) + 14'd1;
    wire [7:0]  rem_minus1 = rem_len - 8'd1;
    wire        last_beat  = (rem_len == 8'd1);

    always @(*) begin
        case (state)
            D_FETCH_D: qspi_cmd_addr = {8'd0, desc_ptr, 2'b00};
            D_READ:    qspi_cmd_addr = {8'd0, src_addr, 2'b00};
            default:   qspi_cmd_addr = {8'd0, dst_addr, 2'b00};
        endcase
    end

    always @(*) begin
        case (req_op)
            OP_DESC_LO,
            OP_DESC_MID:   resp_data16 = {8'd0, req_data};
            OP_DESC_HI:    resp_data16 = 16'd0;
            OP_START:      resp_data16 = {15'd0, (state != D_IDLE)};
            OP_STATUS:     resp_data16 = {6'd0, done_sticky, dma_busy, 2'd0, desc_count};
            OP_DESC_COUNT: resp_data16 = {10'd0, desc_count};
            default:       resp_data16 = 16'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= D_IDLE;
            desc_ptr         <= 14'd0;
            src_addr         <= 14'd0;
            dst_addr         <= 14'd0;
            next_ptr         <= 14'd0;
            rem_len          <= 8'd0;
            flags            <= 12'd0;
            data_buf         <= 32'd0;
            fetch_idx        <= 2'd0;
            q_active         <= 1'b0;
            parity_idx       <= 1'b0;
            pace_armed       <= 1'b0;
            desc_count       <= 6'd0;
            done_sticky      <= 1'b0;

            qspi_cmd_valid   <= 1'b0;
            qspi_cmd_write   <= 1'b0;
            qspi_wdata_valid <= 1'b0;

            sercom_tx_valid  <= 1'b0;
            sercom_rx_ready  <= 1'b0;
            pio_tx_valid     <= 1'b0;
            pio_rx_ready     <= 1'b0;
            pwm0_cc_we       <= 1'b0;
            pwm1_cc_we       <= 1'b0;
            accel_wr_a       <= 1'b0;
            accel_wr_b       <= 1'b0;
            dma_inj_valid    <= 1'b0;
            dma_inj_a        <= 24'd0;

            evt_desc_done    <= 1'b0;
            evt_chain_done   <= 1'b0;
        end else begin
            qspi_cmd_valid  <= 1'b0;
            sercom_tx_valid <= 1'b0;
            sercom_rx_ready <= 1'b0;
            pio_tx_valid    <= 1'b0;
            pio_rx_ready    <= 1'b0;
            pwm0_cc_we      <= 1'b0;
            pwm1_cc_we      <= 1'b0;
            accel_wr_a      <= 1'b0;
            accel_wr_b      <= 1'b0;
            evt_desc_done   <= 1'b0;
            evt_chain_done  <= 1'b0;

            if (pace_pwm0 || pace_pwm1) pace_armed <= 1'b1;

            if (req_valid) begin
                case (req_op)
                    OP_DESC_LO:  desc_ptr[5:0]   <= req_data[7:2];
                    OP_DESC_MID: desc_ptr[13:6]  <= req_data;
                    OP_DESC_HI:  begin end
                    OP_START: begin
                        if (state == D_IDLE) begin
                            desc_count  <= 6'd0;
                            done_sticky <= 1'b0;
                            q_active    <= 1'b0;
                            fetch_idx   <= 2'd0;
                            state       <= D_FETCH_D;
                        end
                    end
                    OP_ABORT: begin
                        state            <= D_IDLE;
                        q_active         <= 1'b0;
                        qspi_wdata_valid <= 1'b0;
                        dma_inj_valid    <= 1'b0;
                    end
                    OP_STATUS: if (req_data[0]) done_sticky <= 1'b0;
                    default: begin end
                endcase
            end else if (task_start && state == D_IDLE) begin
                desc_count  <= 6'd0;
                done_sticky <= 1'b0;
                q_active    <= 1'b0;
                fetch_idx   <= 2'd0;
                state       <= D_FETCH_D;
            end

            case (state)
                D_IDLE: begin
                    q_active   <= 1'b0;
                    pace_armed <= 1'b0;
                end

                // Burst-read 4-word descriptor from PSRAM A
                D_FETCH_D: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_write <= 1'b0;
                            fetch_idx      <= 2'd0;
                            q_active       <= 1'b1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin
                            case (fetch_idx)
                                2'd0: src_addr <= qspi_rdata[15:2];
                                2'd1: dst_addr <= qspi_rdata[15:2];
                                2'd2: begin
                                    rem_len <= qspi_rdata[7:0];
                                    flags   <= qspi_rdata[27:16];
                                end
                                2'd3: next_ptr <= qspi_rdata[15:2];
                            endcase
                            fetch_idx <= fetch_idx + 2'd1;
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                            q_active   <= 1'b0;
                            parity_idx <= 1'b0;
                            pace_armed <= 1'b0;
                            if (rem_len == 8'd0) state <= D_NEXT;
                            else                 state <= D_PACE;
                        end
                    end
                end

                D_PACE: begin
                    if ((!pace_en0 && !(pace_en1 && dst_sel == DST_PWM1)) ||
                        (pace_en0 && (pace_pwm0 || pace_armed)) ||
                        (pace_en1 && dst_sel == DST_PWM1 && (pace_pwm1 || pace_armed))) begin
                        pace_armed <= 1'b0;
                        state      <= D_READ;
                    end
                end

                D_READ: begin
                    case (src_sel)
                        SRC_PSRAM: begin
                            if (!q_active) begin
                                if (!qspi_busy && qspi_cmd_ready) begin
                                    qspi_cmd_valid <= 1'b1;
                                    qspi_cmd_write <= 1'b0;
                                    q_active       <= 1'b1;
                                end
                            end else begin
                                if (qspi_rdata_valid) data_buf <= qspi_rdata;
                                if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                                    q_active <= 1'b0;
                                    if (src_inc) src_addr <= addr_plus1;
                                    state <= D_WRITE;
                                end
                            end
                        end
                        SRC_SERCOM: if (sercom_rx_valid) begin
                            data_buf        <= {24'd0, sercom_rx_data};
                            sercom_rx_ready <= 1'b1;
                            state           <= D_WRITE;
                        end
                        SRC_PIO: if (pio_rx_valid) begin
                            data_buf     <= {16'd0, pio_rx_data};
                            pio_rx_ready <= 1'b1;
                            state        <= D_WRITE;
                        end
                        SRC_ACCEL: if (accel_ready) begin
                            data_buf <= accel_rdata;
                            state    <= D_WRITE;
                        end
                        SRC_TIMER0: begin
                            data_buf <= {16'd0, timer0_cnt};
                            state    <= D_WRITE;
                        end
                        SRC_TIMER1: begin
                            data_buf <= {16'd0, timer1_cnt};
                            state    <= D_WRITE;
                        end
                        SRC_ZERO: begin
                            data_buf <= 32'd0;
                            state    <= D_WRITE;
                        end
                        default: begin
                            data_buf <= 32'd0;
                            state    <= D_WRITE;
                        end
                    endcase
                end

                D_WRITE: begin
                    case (dst_sel)
                        DST_PSRAM: begin
                            if (!q_active) begin
                                if (!qspi_busy && qspi_cmd_ready) begin
                                    qspi_cmd_valid   <= 1'b1;
                                    qspi_cmd_write   <= 1'b1;
                                    qspi_wdata_valid <= 1'b1;
                                    q_active         <= 1'b1;
                                end
                            end else begin
                                if (qspi_wdata_valid && qspi_wdata_ready) qspi_wdata_valid <= 1'b0;
                                if (!qspi_busy && !qspi_cmd_valid) begin
                                    q_active <= 1'b0;
                                    if (dst_inc) dst_addr <= addr_plus1;
                                    rem_len <= rem_minus1;
                                    state   <= last_beat ? D_NEXT : D_PACE;
                                end
                            end
                        end
                        DST_SERCOM: if (sercom_tx_ready) begin
                            sercom_tx_valid <= 1'b1;
                            rem_len         <= rem_minus1;
                            state           <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_PIO: if (pio_tx_ready) begin
                            pio_tx_valid <= 1'b1;
                            rem_len      <= rem_minus1;
                            state        <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_PWM0: begin
                            pwm0_cc_we <= 1'b1;
                            rem_len    <= rem_minus1;
                            state      <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_PWM1: begin
                            pwm1_cc_we <= 1'b1;
                            rem_len    <= rem_minus1;
                            state      <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_ACCEL_AB: if (accel_ready) begin
                            if (!parity_idx) accel_wr_a <= 1'b1;
                            else             accel_wr_b <= 1'b1;
                            parity_idx <= ~parity_idx;
                            rem_len    <= rem_minus1;
                            state      <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_ACCEL_ONE: if (accel_ready) begin
                            if (flags[11]) accel_wr_b <= 1'b1;
                            else           accel_wr_a <= 1'b1;
                            rem_len <= rem_minus1;
                            state   <= last_beat ? D_NEXT : D_PACE;
                        end
                        DST_INET_PAIR: begin
                            if (!parity_idx) begin
                                dma_inj_a  <= data_buf[23:0];
                                parity_idx <= 1'b1;
                                rem_len    <= rem_minus1;
                                state      <= last_beat ? D_NEXT : D_PACE;
                            end else begin
                                dma_inj_valid <= 1'b1;
                                parity_idx    <= 1'b0;
                                state         <= D_INJ_WAIT;
                            end
                        end
                    endcase
                end

                D_INJ_WAIT: begin
                    if (dma_inj_valid && dma_inj_ready) begin
                        dma_inj_valid <= 1'b0;
                        rem_len       <= rem_minus1;
                        state         <= last_beat ? D_NEXT : D_PACE;
                    end
                end

                D_NEXT: begin
                    desc_count    <= desc_count + 6'd1;
                    evt_desc_done <= 1'b1;
                    if (chain_en && (next_ptr != 14'd0)) begin
                        desc_ptr  <= next_ptr;
                        fetch_idx <= 2'd0;
                        state     <= D_FETCH_D;
                    end else begin
                        done_sticky    <= 1'b1;
                        evt_chain_done <= 1'b1;
                        state          <= D_IDLE;
                    end
                end

                default: state <= D_IDLE;
            endcase
        end
    end

endmodule
