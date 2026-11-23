/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compact 4xINT8 SIMD Dot-Product & 16x16 Signed Q15 MAC Accelerator.
 *   - Shares ONE 9x9 signed multiplier executing 4 cycles per MAC:
 *       mode_q15=0: acc += a0*b0 + a1*b1 + a2*b2 + a3*b3  (signed 4xINT8 SIMD)
 *       mode_q15=1: acc += signed(a[15:0]) * signed(b[15:0]) (exact 16x16 Q15 MAC)
 *   - Hardware ReLU, arithmetic right shift, and INT8/Q15 saturation clamp.
 *   - Accessible via INet IO (DEV_ACCEL=7), SWD MMIO, Linked-List DMA, and PPI.
 */

`default_nettype none

module inet_accel (
    input  wire        clk,
    input  wire        rst_n,

    // Register / IO-bridge interface (DEV_ACCEL = 7)
    input  wire        req_valid,
    input  wire [3:0]  req_op,
    input  wire [7:0]  req_data,
    output reg         resp_valid,
    output reg  [15:0] resp_data16,

    // EasyDMA streaming interface
    input  wire        dma_wr_a,
    input  wire        dma_wr_b, // writes op_b and starts 4-cycle MAC
    input  wire [31:0] dma_wdata,
    output wire        dma_ready,
    output wire [31:0] dma_rdata,

    // PPI integration
    input  wire        task_start,
    output reg         evt_done
);

    localparam [3:0] OP_CFG         = 4'd0;
    localparam [3:0] OP_LOAD_A_B0   = 4'd1;
    localparam [3:0] OP_LOAD_A_B1   = 4'd2;
    localparam [3:0] OP_LOAD_A_B2   = 4'd3;
    localparam [3:0] OP_LOAD_A_B3   = 4'd4;
    localparam [3:0] OP_LOAD_B_B0   = 4'd5;
    localparam [3:0] OP_LOAD_B_B1   = 4'd6;
    localparam [3:0] OP_LOAD_B_B2   = 4'd7;
    localparam [3:0] OP_LOAD_B3_RUN = 4'd8;
    localparam [3:0] OP_RUN         = 4'd9;
    localparam [3:0] OP_READ_ACC_LO = 4'd10;
    localparam [3:0] OP_READ_ACC_HI = 4'd11;
    localparam [3:0] OP_READ_QUANT  = 4'd12;
    localparam [3:0] OP_CLEAR       = 4'd13;
    localparam [3:0] OP_STATUS      = 4'd14;

    reg [31:0] op_a, op_b;
    reg signed [31:0] acc;
    reg        cfg_mode_q15, cfg_relu, cfg_auto_clr;
    reg [2:0]  cfg_shift_code;

    function [2:0] encode_shift;
        input [4:0] sh;
        begin
            case (sh)
                5'd0:    encode_shift = 3'd0;
                5'd2:    encode_shift = 3'd1;
                5'd4:    encode_shift = 3'd2;
                5'd6:    encode_shift = 3'd3;
                5'd7:    encode_shift = 3'd4;
                5'd8:    encode_shift = 3'd5;
                5'd12:   encode_shift = 3'd6;
                default: encode_shift = 3'd7;
            endcase
        end
    endfunction

    reg        busy;
    reg [1:0]  step;
    reg        pend_resp;
    reg        clr_on_s0;

    // Shared 9x9 signed multiplier
    reg signed [8:0] mul_x, mul_y;
    always @(*) begin
        mul_x = 9'sd0;
        mul_y = 9'sd0;
        if (!cfg_mode_q15) begin
            case (step)
                2'd0: begin mul_x = {op_a[7],  op_a[7:0]};   mul_y = {op_b[7],  op_b[7:0]};   end
                2'd1: begin mul_x = {op_a[15], op_a[15:8]};  mul_y = {op_b[15], op_b[15:8]};  end
                2'd2: begin mul_x = {op_a[23], op_a[23:16]}; mul_y = {op_b[23], op_b[23:16]}; end
                2'd3: begin mul_x = {op_a[31], op_a[31:24]}; mul_y = {op_b[31], op_b[31:24]}; end
            endcase
        end else begin
            case (step)
                2'd0: begin mul_x = {1'b0, op_a[7:0]};       mul_y = {1'b0, op_b[7:0]};       end
                2'd1: begin mul_x = {1'b0, op_a[7:0]};       mul_y = {op_b[15], op_b[15:8]};  end
                2'd2: begin mul_x = {op_a[15], op_a[15:8]};  mul_y = {1'b0, op_b[7:0]};       end
                2'd3: begin mul_x = {op_a[15], op_a[15:8]};  mul_y = {op_b[15], op_b[15:8]};  end
            endcase
        end
    end

    wire signed [17:0] prod18 = mul_x * mul_y;
    reg  signed [31:0] term32;
    always @(*) begin
        if (!cfg_mode_q15) begin
            term32 = {{14{prod18[17]}}, prod18};
        end else begin
            case (step)
                2'd0:    term32 = {{14{prod18[17]}}, prod18};
                2'd1,
                2'd2:    term32 = {{6{prod18[17]}},  prod18, 8'd0};
                default: term32 = {prod18[15:0], 16'd0};
            endcase
        end
    end

    // Hardware ReLU + compact 8-way shift (supports INT8/Q15 shifts 0,2,4,6,7,8,12,15) + bit-reduction saturation
    wire signed [31:0] val_relu = (cfg_relu && acc[31]) ? 32'sd0 : acc;
    reg  signed [31:0] val_sh;
    always @(*) begin
        case (cfg_shift_code)
            3'd0:    val_sh = val_relu;
            3'd1:    val_sh = val_relu >>> 2;
            3'd2:    val_sh = val_relu >>> 4;
            3'd3:    val_sh = val_relu >>> 6;
            3'd4:    val_sh = val_relu >>> 7;
            3'd5:    val_sh = val_relu >>> 8;
            3'd6:    val_sh = val_relu >>> 12;
            default: val_sh = val_relu >>> 15;
        endcase
    end

    function [15:0] sat_quant;
        input signed [31:0] v;
        input               int8_mode;
        reg                 pos_ovf, neg_ovf;
        begin
            if (int8_mode) begin
                pos_ovf = !v[31] && (|v[30:7]);
                neg_ovf =  v[31] && !(&v[30:7]);
                if (pos_ovf)      sat_quant = 16'h007F;
                else if (neg_ovf) sat_quant = 16'h0080;
                else              sat_quant = {8'd0, v[7:0]};
            end else begin
                pos_ovf = !v[31] && (|v[30:15]);
                neg_ovf =  v[31] && !(&v[30:15]);
                if (pos_ovf)      sat_quant = 16'h7FFF;
                else if (neg_ovf) sat_quant = 16'h8000;
                else              sat_quant = v[15:0];
            end
        end
    endfunction

    assign dma_ready = !busy;
    assign dma_rdata = acc;

    always @(*) begin
        case (req_op)
            OP_READ_ACC_LO,
            OP_LOAD_B3_RUN,
            OP_RUN:         resp_data16 = acc[15:0];
            OP_READ_ACC_HI: resp_data16 = acc[31:16];
            OP_READ_QUANT:  resp_data16 = sat_quant(val_sh, req_data[0]);
            OP_STATUS:      resp_data16 = {15'd0, busy};
            OP_CLEAR:       resp_data16 = 16'd0;
            default:        resp_data16 = {8'd0, req_data};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_a           <= 32'd0;
            op_b           <= 32'd0;
            acc            <= 32'sd0;
            cfg_mode_q15   <= 1'b0;
            cfg_relu       <= 1'b0;
            cfg_auto_clr   <= 1'b0;
            cfg_shift_code <= 3'd0;
            busy           <= 1'b0;
            step           <= 2'd0;
            pend_resp      <= 1'b0;
            clr_on_s0      <= 1'b0;
            resp_valid     <= 1'b0;
            evt_done       <= 1'b0;
        end else begin
            resp_valid <= 1'b0;
            evt_done   <= 1'b0;

            if (busy) begin : mac_step
                reg signed [31:0] next_acc;
                next_acc = (clr_on_s0 ? 32'sd0 : acc) + term32;
                acc       <= next_acc;
                clr_on_s0 <= 1'b0;
                if (step == 2'd3) begin
                    busy      <= 1'b0;
                    evt_done  <= 1'b1;
                    if (pend_resp) begin
                        pend_resp  <= 1'b0;
                        resp_valid <= 1'b1;
                    end
                end else begin
                    step <= step + 2'd1;
                end
            end else begin
                if (dma_wr_a) op_a <= dma_wdata;
                if (dma_wr_b) begin
                    op_b      <= dma_wdata;
                    busy      <= 1'b1;
                    step      <= 2'd0;
                    clr_on_s0 <= cfg_auto_clr;
                    pend_resp <= 1'b0;
                end else if (task_start) begin
                    busy      <= 1'b1;
                    step      <= 2'd0;
                    clr_on_s0 <= cfg_auto_clr;
                    pend_resp <= 1'b0;
                end else if (req_valid) begin
                    case (req_op)
                        OP_CFG: begin
                            cfg_shift_code <= encode_shift(req_data[7:3]);
                            cfg_auto_clr   <= req_data[2];
                            cfg_relu       <= req_data[1];
                            cfg_mode_q15   <= req_data[0];
                            resp_valid     <= 1'b1;
                        end
                        OP_LOAD_A_B0: begin op_a[7:0]   <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_A_B1: begin op_a[15:8]  <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_A_B2: begin op_a[23:16] <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_A_B3: begin op_a[31:24] <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_B_B0: begin op_b[7:0]   <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_B_B1: begin op_b[15:8]  <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_B_B2: begin op_b[23:16] <= req_data; resp_valid <= 1'b1; end
                        OP_LOAD_B3_RUN: begin
                            op_b[31:24] <= req_data;
                            busy        <= 1'b1;
                            step        <= 2'd0;
                            clr_on_s0   <= cfg_auto_clr;
                            pend_resp   <= 1'b1;
                        end
                        OP_RUN: begin
                            busy      <= 1'b1;
                            step      <= 2'd0;
                            clr_on_s0 <= req_data[0] | cfg_auto_clr;
                            pend_resp <= 1'b1;
                        end
                        OP_CLEAR: begin acc <= 32'sd0; resp_valid <= 1'b1; end
                        default:  resp_valid <= 1'b1;
                    endcase
                end
            end
        end
    end

endmodule
