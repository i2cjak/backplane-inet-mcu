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

    wire [3:0]  sh   = b[3:0];
    wire [15:0] prod = a * b;

    always @(*) begin
        case (op)
            ALU_ADD:  res = a + b;
            ALU_SUB:  res = a - b;
            ALU_AND:  res = a & b;
            ALU_OR:   res = a | b;
            ALU_XOR:  res = a ^ b;
            ALU_SHL:  res = a << sh;
            ALU_SHR:  res = a >> sh;
            ALU_SAR:  res = $unsigned($signed(a) >>> sh);
            ALU_EQ:   res = {15'd0, (a == b)};
            ALU_NE:   res = {15'd0, (a != b)};
            ALU_LTU:  res = {15'd0, (a < b)};
            ALU_LTS:  res = {15'd0, ($signed(a) < $signed(b))};
            ALU_LEU:  res = {15'd0, (a <= b)};
            ALU_MUL:  res = prod;
            ALU_SEQ:  res = b;
            ALU_PACK: res = {a[7:0], b[7:0]};
            default:  res = 16'd0;
        endcase
    end

endmodule
