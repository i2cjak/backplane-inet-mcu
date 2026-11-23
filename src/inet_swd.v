/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * ARM ADIv5 Serial Wire Debug (SWD) target synchronously oversampled in clk.
 * Supports:
 *   - >=50-cycle line reset (and JTAG-to-SWD 0xE79E + line reset)
 *   - 8-bit header parity check, 1-cycle turnaround, 3-bit ACK (OK/WAIT/FAULT)
 *   - 32-bit LSB-first R/W data + even parity
 *   - DP: 0x0 IDCODE/ABORT, 0x4 CTRL/STAT, 0x8 SELECT, 0xC RDBUFF
 *   - AP Bank 0: 0x0 DBG_CTRL, 0x4 DBG_ADDR, 0x8 DBG_DATA (posted read->RDBUFF), 0xC IDR
 *   - AP Bank 1: 0x0 cycle_cnt, 0x4 redex_cnt, 0x8 qspi_txn_cnt, 0xC queue_depths
 */

`default_nettype none

module inet_swd (
    input  wire        clk,
    input  wire        rst_n,

    // Physical SWD pins (SWCLK oversampled in clk)
    input  wire        swclk_in,
    input  wire        swdio_in,
    output reg         swdio_out,
    output reg         swdio_oe,

    // Boot / run control & core status
    input  wire        boot_halt,
    output reg         dbg_halt_req,
    output reg         dbg_step_pulse,
    output reg         dbg_reset_pulse,
    input  wire        core_halted,
    input  wire        core_busy,
    input  wire        qspi_busy,
    input  wire [3:0]  active_tiles,

    // Telemetry counters (AP Bank 1)
    input  wire [31:0] cycle_cnt,
    input  wire [31:0] redex_cnt,
    input  wire [31:0] qspi_txn_cnt,
    input  wire [31:0] queue_depths,

    // Debug memory/node/PSRAM access port
    output reg         dbg_mem_req,
    output reg         dbg_mem_we,
    output reg  [23:0] dbg_mem_addr,
    output reg  [31:0] dbg_mem_wdata,
    input  wire        dbg_mem_ready,
    input  wire [31:0] dbg_mem_rdata
);

    localparam [31:0] DP_IDCODE_VAL = 32'h11E7_0001;
    localparam [31:0] AP_IDR_VAL    = 32'h11E7_0010;

    localparam [2:0] ACK_OK    = 3'b001;
    localparam [2:0] ACK_WAIT  = 3'b010;
    localparam [2:0] ACK_FAULT = 3'b100;

    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_HDR       = 4'd1;
    localparam [3:0] ST_TRN_IN    = 4'd2;
    localparam [3:0] ST_TRN_DRIVE = 4'd8;
    localparam [3:0] ST_ACK       = 4'd3;
    localparam [3:0] ST_RDATA     = 4'd4;
    localparam [3:0] ST_RPAR      = 4'd5;
    localparam [3:0] ST_TRN_OUT   = 4'd6;
    localparam [3:0] ST_WDATA     = 4'd7; // + bit_cnt==32 serves as WPAR
    localparam [3:0] ST_TRN_END   = 4'd9;

    // Input synchronizers
    reg [2:0] swclk_sync;
    reg [1:0] swdio_sync;
    wire swclk_rise = swclk_sync[1] && !swclk_sync[2];
    wire swclk_fall = !swclk_sync[1] && swclk_sync[2];
    wire swdio_s    = swdio_sync[1];

    // Protocol FSM
    reg [3:0]  state;
    reg [5:0]  bit_cnt;
    reg [5:0]  ones_cnt;
    reg [6:0]  hdr_shift;
    reg [2:0]  ack_bits;
    reg [31:0] data_shift;
    reg        hdr_apndp;
    reg        hdr_rnw;
    reg [1:0]  hdr_addr;

    // DP / AP registers
    reg        csyspwrupreq;
    reg        cdbgpwrupreq;
    reg        stickyerr;
    reg [3:0]  ap_banksel;
    reg [31:0] ap_rdbuff;

    wire [31:0] dbg_ctrl_stat = {
        24'd0,
        active_tiles,
        1'b0,
        qspi_busy,
        core_busy,
        core_halted | boot_halt
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            swclk_sync      <= 3'b000;
            swdio_sync      <= 2'b11;
            swdio_out       <= 1'b1;
            swdio_oe        <= 1'b0;
            state           <= ST_IDLE;
            bit_cnt         <= 6'd0;
            ones_cnt        <= 6'd0;
            hdr_shift       <= 7'd0;
            ack_bits        <= 3'd0;
            data_shift      <= 32'd0;
            hdr_apndp       <= 1'b0;
            hdr_rnw         <= 1'b0;
            hdr_addr        <= 2'd0;

            csyspwrupreq    <= 1'b0;
            cdbgpwrupreq    <= 1'b0;
            stickyerr       <= 1'b0;
            ap_banksel      <= 4'd0;
            ap_rdbuff       <= 32'd0;

            dbg_halt_req    <= 1'b0;
            dbg_step_pulse  <= 1'b0;
            dbg_reset_pulse <= 1'b0;
            dbg_mem_req     <= 1'b0;
            dbg_mem_we      <= 1'b0;
            dbg_mem_addr    <= 24'd0;
            dbg_mem_wdata   <= 32'd0;
        end else begin
            swclk_sync      <= {swclk_sync[1:0], swclk_in};
            swdio_sync      <= {swdio_sync[0], swdio_in};
            dbg_step_pulse  <= 1'b0;
            dbg_reset_pulse <= 1'b0;

            // Complete posted AP memory transaction
            if (dbg_mem_req && dbg_mem_ready) begin
                dbg_mem_req <= 1'b0;
                if (!dbg_mem_we) begin
                    ap_rdbuff <= dbg_mem_rdata;
                end
            end

            // Track consecutive 1s on SWCLK rising edges for line reset (>=50)
            if (swclk_rise) begin
                if (swdio_s && !swdio_oe) begin
                    if (ones_cnt < 6'd63) begin
                        ones_cnt <= ones_cnt + 6'd1;
                    end
                end else begin
                    ones_cnt <= 6'd0;
                end
            end

            // Line reset overrides FSM immediately once >=50 consecutive 1s seen
            if (swclk_rise && (ones_cnt >= 6'd49) && swdio_s && !swdio_oe) begin
                state    <= ST_IDLE;
                swdio_oe <= 1'b0;
                bit_cnt  <= 6'd0;
            end else begin
                // ---------------- Sample on SWCLK rising edge ----------------
                if (swclk_rise) begin
                    case (state)
                        ST_IDLE: begin
                            if (swdio_s) begin
                                hdr_shift <= {swdio_s, 6'd0};
                                bit_cnt   <= 6'd1;
                                state     <= ST_HDR;
                            end
                        end

                        ST_HDR: begin : hdr_blk
                            reg [7:0] full_hdr;
                            reg       h_par;
                            reg       h_ok;
                            reg       ap_sel;
                            reg       rw_sel;
                            reg [1:0] a_sel;
                            full_hdr = {swdio_s, hdr_shift};
                            hdr_shift <= full_hdr[7:1];
                            if (bit_cnt == 6'd7) begin
                                // full_hdr[0]=Start(1), [1]=APnDP, [2]=RnW, [3]=A2, [4]=A3,
                                // [5]=Parity, [6]=Stop(0), [7]=Park(1)
                                ap_sel = full_hdr[1];
                                rw_sel = full_hdr[2];
                                a_sel  = {full_hdr[4], full_hdr[3]};
                                h_par  = ap_sel ^ rw_sel ^ a_sel[0] ^ a_sel[1];
                                h_ok   = (full_hdr[0] == 1'b1) &&
                                         (full_hdr[5] == h_par) &&
                                         (full_hdr[6] == 1'b0) &&
                                         (full_hdr[7] == 1'b1);
                                if (!h_ok) begin
                                    state <= ST_IDLE;
                                end else begin
                                    hdr_apndp <= ap_sel;
                                    hdr_rnw   <= rw_sel;
                                    hdr_addr  <= a_sel;
                                    state     <= ST_TRN_IN;

                                    // Compute ACK and pre-latch read data
                                    if (stickyerr && !(ap_sel == 1'b0 && rw_sel == 1'b0 && a_sel == 2'd0)) begin
                                        ack_bits <= ACK_FAULT;
                                    end else if (dbg_mem_req &&
                                                ((ap_sel == 1'b1 && a_sel == 2'd2) ||
                                                 (ap_sel == 1'b0 && rw_sel == 1'b1 && a_sel == 2'd3))) begin
                                        ack_bits <= ACK_WAIT;
                                    end else begin
                                        ack_bits <= ACK_OK;
                                        if (rw_sel) begin
                                            if (ap_sel == 1'b0) begin
                                                case (a_sel)
                                                    2'd0: data_shift <= DP_IDCODE_VAL;
                                                    2'd1: data_shift <= {csyspwrupreq, csyspwrupreq,
                                                                         cdbgpwrupreq, cdbgpwrupreq,
                                                                         22'd0, stickyerr, 5'd0};
                                                    2'd2: data_shift <= {24'd0, ap_banksel, 4'd0};
                                                    2'd3: data_shift <= ap_rdbuff;
                                                endcase
                                            end else begin
                                                if (ap_banksel == 4'd0) begin
                                                    case (a_sel)
                                                        2'd0: data_shift <= dbg_ctrl_stat;
                                                        2'd1: data_shift <= {8'd0, dbg_mem_addr};
                                                        2'd2: begin
                                                            data_shift <= ap_rdbuff;
                                                            // Trigger posted AP memory read
                                                            dbg_mem_req <= 1'b1;
                                                            dbg_mem_we  <= 1'b0;
                                                        end
                                                        2'd3: data_shift <= AP_IDR_VAL;
                                                    endcase
                                                end else if (ap_banksel == 4'd1) begin
                                                    case (a_sel)
                                                        2'd0: data_shift <= cycle_cnt;
                                                        2'd1: data_shift <= redex_cnt;
                                                        2'd2: data_shift <= qspi_txn_cnt;
                                                        2'd3: data_shift <= queue_depths;
                                                    endcase
                                                end else begin
                                                    data_shift <= 32'd0;
                                                end
                                            end
                                        end
                                    end
                                end
                            end else begin
                                bit_cnt <= bit_cnt + 6'd1;
                            end
                        end

                        ST_TRN_IN: begin
                            state <= ST_TRN_DRIVE;
                        end

                        ST_TRN_END: begin
                            state <= ST_IDLE;
                        end

                        ST_TRN_OUT: begin
                            bit_cnt <= 6'd0;
                            state   <= ST_WDATA;
                        end

                        ST_WDATA: begin
                            if (bit_cnt < 6'd32) begin
                                data_shift <= {swdio_s, data_shift[31:1]};
                                bit_cnt    <= bit_cnt + 6'd1;
                            end else begin
                                // Parity bit check
                                if (swdio_s != (^data_shift)) begin
                                    stickyerr <= 1'b1;
                                end else begin
                                    if (hdr_apndp == 1'b0) begin
                                        case (hdr_addr)
                                            2'd0: begin
                                                // ABORT clears sticky error flags
                                                if (data_shift[0] || data_shift[2]) begin
                                                    stickyerr <= 1'b0;
                                                end
                                            end
                                            2'd1: begin
                                                csyspwrupreq <= data_shift[30];
                                                cdbgpwrupreq <= data_shift[28];
                                            end
                                            2'd2: begin
                                                ap_banksel <= data_shift[7:4];
                                            end
                                            default: begin end
                                        endcase
                                    end else if (ap_banksel == 4'd0) begin
                                        case (hdr_addr)
                                            2'd0: begin
                                                dbg_halt_req    <= data_shift[0];
                                                dbg_step_pulse  <= data_shift[1];
                                                dbg_reset_pulse <= data_shift[2];
                                            end
                                            2'd1: begin
                                                dbg_mem_addr <= data_shift[23:0];
                                            end
                                            2'd2: begin
                                                dbg_mem_wdata <= data_shift;
                                                dbg_mem_we    <= 1'b1;
                                                dbg_mem_req   <= 1'b1;
                                            end
                                            default: begin end
                                        endcase
                                    end
                                end
                                state <= ST_IDLE;
                            end
                        end

                        default: begin end
                    endcase
                end

                // ---------------- Drive on SWCLK falling edge ----------------
                if (swclk_fall) begin
                    case (state)
                        ST_TRN_DRIVE: begin
                            swdio_oe  <= 1'b1;
                            swdio_out <= ack_bits[0];
                            bit_cnt   <= 6'd0;
                            state     <= ST_ACK;
                        end

                        ST_ACK: begin
                            if (bit_cnt < 6'd2) begin
                                swdio_out <= ack_bits[bit_cnt[1:0] + 2'd1];
                                bit_cnt   <= bit_cnt + 6'd1;
                            end else if (ack_bits != ACK_OK) begin
                                swdio_oe <= 1'b0;
                                state    <= ST_TRN_END;
                            end else if (hdr_rnw) begin
                                swdio_out <= data_shift[0];
                                bit_cnt   <= 6'd0;
                                state     <= ST_RDATA;
                            end else begin
                                swdio_oe <= 1'b0;
                                state    <= ST_TRN_OUT;
                            end
                        end

                        ST_RDATA: begin
                            if (bit_cnt < 6'd31) begin
                                swdio_out <= data_shift[bit_cnt[4:0] + 5'd1];
                                bit_cnt   <= bit_cnt + 6'd1;
                            end else begin
                                swdio_out <= ^data_shift; // even parity
                                state     <= ST_RPAR;
                            end
                        end

                        ST_RPAR: begin
                            swdio_oe <= 1'b0;
                            state    <= ST_TRN_END;
                        end

                        default: begin end
                    endcase
                end
            end
        end
    end

endmodule
