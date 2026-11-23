/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Glitch-free active-low latch-based Register File for SKY130 HD.
 *   - Registers {we, waddr, wdata} on posedge clk so write address & data are
 *     completely stable before the active-low latch transparent phase (clk == 0).
 *   - Active-low gated write enables (gclk_n = clk | ~we_oh_q) infer $_DLATCH_N_
 *     which maps cleanly to legal sky130_fd_sc_hd__dlxtn_1 cells (no lpflow).
 *   - Optional 1-entry RAW bypass (BYPASS=1) provides exact 1-cycle-write /
 *     0-cycle-read semantics matching a synchronous-write, combinational-read DFF array.
 */

`default_nettype none

module inet_latch_rf #(
    parameter integer DEPTH     = 32,
    parameter integer WIDTH     = 48,
    parameter integer ADDR_W    = 5,
    parameter integer BYPASS    = 1,
    parameter integer REG_INPUT = 0
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              we,
    input  wire [ADDR_W-1:0] waddr,
    input  wire [WIDTH-1:0]  wdata,
    input  wire [ADDR_W-1:0] raddr,
    output wire [WIDTH-1:0]  rdata
);

    wire              eff_we;
    wire [ADDR_W-1:0] eff_waddr;
    wire [WIDTH-1:0]  eff_wdata;

    generate
        if (REG_INPUT != 0) begin : gen_reg_in
            reg              we_q;
            reg [ADDR_W-1:0] waddr_q;
            reg [WIDTH-1:0]  wdata_q;
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    we_q    <= 1'b0;
                    waddr_q <= {ADDR_W{1'b0}};
                    wdata_q <= {WIDTH{1'b0}};
                end else begin
                    we_q <= we;
                    if (we) begin
                        waddr_q <= waddr;
                        wdata_q <= wdata;
                    end
                end
            end
            assign eff_we    = we_q;
            assign eff_waddr = waddr_q;
            assign eff_wdata = wdata_q;
        end else begin : gen_direct_in
            wire _unused_rst = rst_n;
            assign eff_we    = we;
            assign eff_waddr = waddr;
            assign eff_wdata = wdata;
        end
    endgenerate

    wire [DEPTH-1:0] we_oh_q = eff_we ? ({{(DEPTH-1){1'b0}}, 1'b1} << eff_waddr) : {DEPTH{1'b0}};
    wire [DEPTH-1:0] gclk_n  = {DEPTH{clk}} | ~we_oh_q;

    reg [WIDTH-1:0] mem [0:DEPTH-1];

`ifndef SYNTHESIS
    integer init_i;
    initial begin
        for (init_i = 0; init_i < DEPTH; init_i = init_i + 1) begin
            mem[init_i] = {WIDTH{1'b0}};
        end
    end
`endif

    integer idx_i;
    /* verilator lint_off LATCH */
    always @(*) begin
        for (idx_i = 0; idx_i < DEPTH; idx_i = idx_i + 1) begin
            if (!gclk_n[idx_i]) begin
                mem[idx_i] = eff_wdata;
            end
        end
    end
    /* verilator lint_on LATCH */

    generate
        if (BYPASS != 0) begin : gen_bypass
            assign rdata = (eff_we && (raddr == eff_waddr)) ? eff_wdata : mem[raddr];
        end else begin : gen_nobypass
            assign rdata = mem[raddr];
        end
    endgenerate

endmodule
