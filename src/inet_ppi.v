/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * 4-Channel Programmable Peripheral Interconnect (Nordic PPI/DPPI style).
 *   - Routes 12 hardware events autonomously to 8 hardware tasks with zero core cycles.
 *   - Includes 24-bit (pair_a, pair_b) registers for TASK_INET_INJECT, allowing
 *     timers, DMA, PIO, or GPIO edges to spawn interaction-net active pairs directly.
 */

`default_nettype none

module inet_ppi (
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (DEV_PPI = 5)
    input  wire        req_valid,
    input  wire [3:0]  req_op,
    input  wire [7:0]  req_data,
    output reg  [15:0] resp_data16,

    // Hardware events [11:0] (11=GPIO_EDGE handled via ppi_gpio_in)
    input  wire [10:0] evt_in_ext,
    input  wire        ppi_gpio_in,

    // Hardware task pulses [7:0]
    output wire [7:0]  task_out,

    // Autonomous active-pair injection to inet_core
    output wire        ppi_inj_valid,
    input  wire        ppi_inj_ready,
    output wire [23:0] ppi_inj_a,
    output wire [23:0] ppi_inj_b
);

    reg [3:0]  ch_en;
    reg [15:0] ch_eep_p;
    reg [11:0] ch_tep_p;

    reg [11:0] evt_sticky;
    reg [2:0]  gpio_sync;
    reg [7:0]  sw_task_pulse;

    reg [23:0] inj_pair_a, inj_pair_b;
    reg        inj_pending;

    wire gpio_rise   = gpio_sync[1] && !gpio_sync[2];
    wire [11:0] evt_all = {gpio_rise, evt_in_ext};

    reg [7:0] ppi_task_comb;
    integer ci;
    always @(*) begin : ppi_comb
        reg [3:0] eep;
        reg [2:0] tep;
        ppi_task_comb = sw_task_pulse;
        for (ci = 0; ci < 4; ci = ci + 1) begin
            eep = ch_eep_p[ci*4 +: 4];
            tep = ch_tep_p[ci*3 +: 3];
            if (ch_en[ci] && (eep < 4'd12) && evt_all[eep]) begin
                ppi_task_comb[tep] = 1'b1;
            end
        end
    end

    assign task_out      = ppi_task_comb;
    assign ppi_inj_valid = inj_pending;
    assign ppi_inj_a     = inj_pair_a;
    assign ppi_inj_b     = inj_pair_b;

    always @(*) begin
        case (req_op)
            4'd0, 4'd1, 4'd2, 4'd3,
            4'd4, 4'd5, 4'd6,
            4'd7, 4'd8, 4'd9: resp_data16 = {8'd0, req_data};
            4'd10:            resp_data16 = {13'd0, req_data[2:0]};
            4'd11:            resp_data16 = {ch_en, evt_sticky};
            default:          resp_data16 = 16'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ch_en         <= 4'd0;
            ch_eep_p      <= 16'd0;
            ch_tep_p      <= 12'd0;
            evt_sticky    <= 12'd0;
            gpio_sync     <= 3'd0;
            sw_task_pulse <= 8'd0;
            inj_pair_a    <= 24'd0;
            inj_pair_b    <= 24'd0;
            inj_pending   <= 1'b0;
        end else begin
            gpio_sync     <= {gpio_sync[1:0], ppi_gpio_in};
            sw_task_pulse <= 8'd0;
            evt_sticky    <= evt_sticky | evt_all;

            if (inj_pending && ppi_inj_ready) begin
                inj_pending <= 1'b0;
            end
            if (ppi_task_comb[7]) begin
                inj_pending <= 1'b1;
            end

            if (req_valid) begin
                case (req_op)
                    4'd0, 4'd1, 4'd2, 4'd3: begin
                        ch_en[req_op[1:0]]            <= req_data[7];
                        ch_tep_p[req_op[1:0]*3 +: 3]  <= req_data[6:4];
                        ch_eep_p[req_op[1:0]*4 +: 4]  <= req_data[3:0];
                    end
                    4'd4:  inj_pair_a[7:0]   <= req_data;
                    4'd5:  inj_pair_a[15:8]  <= req_data;
                    4'd6:  inj_pair_a[23:16] <= req_data;
                    4'd7:  inj_pair_b[7:0]   <= req_data;
                    4'd8:  inj_pair_b[15:8]  <= req_data;
                    4'd9:  inj_pair_b[23:16] <= req_data;
                    4'd10: sw_task_pulse <= (8'd1 << req_data[2:0]);
                    4'd11: if (req_data[0]) evt_sticky <= evt_all;
                    default: begin end
                endcase
            end
        end
    end

endmodule
