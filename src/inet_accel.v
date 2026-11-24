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
    reg [1:0]  cfg_shift_code;

    function [1:0] encode_shift;
        input [4:0] sh;
        begin
            case (sh)
                5'd0:    encode_shift = 2'd0;
                5'd7:    encode_shift = 2'd1;
                5'd8:    encode_shift = 2'd2;
                default: encode_shift = 2'd3;
            endcase
        end
    endfunction

    reg        busy;
    reg [4:0]  step;
    reg        pend_resp;
    reg        clr_on_s0;

    // Bit-serial signed shift-add MAC (zero combinational multipliers):
    //   mode_q15=0: 4xINT8 SIMD in 32 cycles (8 cycles per lane)
    //   mode_q15=1: 16x16 Q15 in 16 cycles
    wire [7:0] lane_a = (step[4:3] == 2'd0) ? op_a[7:0]   :
                        (step[4:3] == 2'd1) ? op_a[15:8]  :
                        (step[4:3] == 2'd2) ? op_a[23:16] : op_a[31:24];
    wire [7:0] lane_b = (step[4:3] == 2'd0) ? op_b[7:0]   :
                        (step[4:3] == 2'd1) ? op_b[15:8]  :
                        (step[4:3] == 2'd2) ? op_b[23:16] : op_b[31:24];
    wire       is_sign  = cfg_mode_q15 ? (step[3:0] == 4'd15) : (step[2:0] == 3'd7);
    wire signed [31:0] init_mcand = cfg_mode_q15 ? {{16{op_a[15]}}, op_a[15:0]}
                                                 : {{24{lane_a[7]}}, lane_a};
    wire [3:0]         mcand_sh   = cfg_mode_q15 ? step[3:0] : {1'b0, step[2:0]};
    wire signed [31:0] cur_mcand  = init_mcand <<< mcand_sh;
    wire               cur_b      = cfg_mode_q15 ? op_b[{1'b0, step[3:0]}] : lane_b[step[2:0]];
    wire signed [31:0] base_acc   = clr_on_s0 ? 32'sd0 : acc;
    wire signed [31:0] add_b      = is_sign ? ~cur_mcand : cur_mcand;
    wire signed [31:0] sum_acc    = base_acc + add_b + {31'd0, is_sign};

    // Hardware ReLU + compact 4-way shift (0, 7, 8, 15) + bit-reduction saturation
    wire signed [31:0] val_relu = (cfg_relu && acc[31]) ? 32'sd0 : acc;
    reg  signed [31:0] val_sh;
    always @(*) begin
        case (cfg_shift_code)
            2'd0:    val_sh = val_relu;
            2'd1:    val_sh = val_relu >>> 7;
            2'd2:    val_sh = val_relu >>> 8;
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
            cfg_shift_code <= 2'd0;
            busy           <= 1'b0;
            step           <= 5'd0;
            pend_resp      <= 1'b0;
            clr_on_s0      <= 1'b0;
            resp_valid     <= 1'b0;
            evt_done       <= 1'b0;
        end else begin
            resp_valid <= 1'b0;
            evt_done   <= 1'b0;

            if (busy) begin : mac_step
                acc       <= cur_b ? sum_acc : base_acc;
                clr_on_s0 <= 1'b0;
                if (cfg_mode_q15 ? (step == 5'd15) : (step == 5'd31)) begin
                    busy      <= 1'b0;
                    evt_done  <= 1'b1;
                    if (pend_resp) begin
                        pend_resp  <= 1'b0;
                        resp_valid <= 1'b1;
                    end
                end else begin
                    step <= step + 5'd1;
                end
            end else begin
                if (dma_wr_a) op_a <= dma_wdata;
                if (dma_wr_b) begin
                    op_b      <= dma_wdata;
                    busy      <= 1'b1;
                    step      <= 5'd0;
                    clr_on_s0 <= cfg_auto_clr;
                    pend_resp <= 1'b0;
                end else if (task_start) begin
                    busy      <= 1'b1;
                    step      <= 5'd0;
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
                            step        <= 5'd0;
                            clr_on_s0   <= cfg_auto_clr;
                            pend_resp   <= 1'b1;
                        end
                        OP_RUN: begin
                            busy      <= 1'b1;
                            step      <= 5'd0;
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
