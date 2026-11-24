/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * 16-bit combinational ALU matching spec/isa.py eval_alu() for all 16 opcodes.
 */

`default_nettype none

module inet_alu (
    input  wire [3:0]  op,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] res
);

    localparam [3:0] ALU_ADD  = 4'h0;
    localparam [3:0] ALU_SUB  = 4'h1;
    localparam [3:0] ALU_AND  = 4'h2;
    localparam [3:0] ALU_OR   = 4'h3;
    localparam [3:0] ALU_XOR  = 4'h4;
    localparam [3:0] ALU_SHL  = 4'h5;
    localparam [3:0] ALU_SHR  = 4'h6;
    localparam [3:0] ALU_SAR  = 4'h7;
    localparam [3:0] ALU_EQ   = 4'h8;
    localparam [3:0] ALU_NE   = 4'h9;
    localparam [3:0] ALU_LTU  = 4'hA;
    localparam [3:0] ALU_LTS  = 4'hB;
    localparam [3:0] ALU_LEU  = 4'hC;
    localparam [3:0] ALU_MUL  = 4'hD;
    localparam [3:0] ALU_SEQ  = 4'hE;
    localparam [3:0] ALU_PACK = 4'hF;

    function [15:0] bitrev16;
        input [15:0] v;
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1) bitrev16[k] = v[15 - k];
        end
    endfunction

    wire [3:0]        sh        = b[3:0];
    wire [15:0]       prod      = a * b;
    wire [16:0]       sub17     = {1'b0, a} - {1'b0, b};
    wire [15:0]       sh_in     = (op == ALU_SHL) ? bitrev16(a) : a;
    wire              sh_fill   = (op == ALU_SAR) ? a[15] : 1'b0;
    wire signed [16:0] sh_res17 = $signed({sh_fill, sh_in}) >>> sh;
    wire [15:0]       sh_out    = sh_res17[15:0];
    wire              _unused_s = sh_res17[16];
    wire              eq_flag   = (sub17[15:0] == 16'd0);
    wire              lts_flag  = (a[15] != b[15]) ? a[15] : sub17[15];

    always @(*) begin
        case (op)
            ALU_ADD:  res = a + b;
            ALU_SUB:  res = sub17[15:0];
            ALU_AND:  res = a & b;
            ALU_OR:   res = a | b;
            ALU_XOR:  res = a ^ b;
            ALU_SHL:  res = bitrev16(sh_out);
            ALU_SHR,
            ALU_SAR:  res = sh_out;
            ALU_EQ:   res = {15'd0, eq_flag};
            ALU_NE:   res = {15'd0, !eq_flag};
            ALU_LTU:  res = {15'd0, sub17[16]};
            ALU_LTS:  res = {15'd0, lts_flag};
            ALU_LEU:  res = {15'd0, sub17[16] || eq_flag};
            ALU_MUL:  res = prod;
            ALU_SEQ:  res = b;
            ALU_PACK: res = {a[7:0], b[7:0]};
            default:  res = 16'd0;
        endcase
    end

endmodule
