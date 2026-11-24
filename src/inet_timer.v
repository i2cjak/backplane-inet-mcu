/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Dual 16-bit Hardware Timer / Counter / PWM Engine (Nordic TIMER/PWM style).
 *   - 2 independent 16-bit timers (Timer0 / Timer1) sharing a 15-bit prescaler.
 *   - Each channel generates hardware PWM: pwm_out = en ? ((cnt < cc) ^ pwm_pol) : pwm_pol.
 *   - Fires 1-cycle PPI events on compare (cnt == cc) and period reload (cnt == top),
 *     enabling zero-CPU EasyDMA PWM waveform updates via dma_cc{0,1}_we.
 */

`default_nettype none

module inet_timer (
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (synchronous 1-cycle response via io_bridge / SWD MMIO)
    input  wire        req_valid,
    input  wire [3:0]  req_op,      // [3]=timer_sel (0/1), [2:0]=subop
    input  wire [7:0]  req_data,
    output reg  [15:0] resp_data16,

    // PPI tasks & external capture/clock
    input  wire        t0_task_start_clr,
    input  wire        t0_task_stop,
    input  wire        t1_task_count,
    input  wire        t1_task_start_clr,
    input  wire        ext_cap_in,

    // PPI events (1-cycle pulses)
    output reg         evt_t0_cmp,
    output reg         evt_t0_period,
    output reg         evt_t1_cmp,
    output reg         evt_t1_period,

    // Hardware PWM outputs
    output wire        pwm0_out,
    output wire        pwm1_out,

    // Direct EasyDMA compare/counter access
    input  wire        dma_cc0_we,
    input  wire        dma_cc1_we,
    input  wire [15:0] dma_cc_wdata,
    output wire [15:0] dma_cnt0_rdata,
    output wire [15:0] dma_cnt1_rdata
);

    localparam [2:0] OP_CFG        = 3'd0;
    localparam [2:0] OP_SET_CC_LO  = 3'd1;
    localparam [2:0] OP_SET_CC_HI  = 3'd2;
    localparam [2:0] OP_SET_TOP_LO = 3'd3;
    localparam [2:0] OP_SET_TOP_HI = 3'd4;
    localparam [2:0] OP_READ_CNT   = 3'd5;
    localparam [2:0] OP_CTRL       = 3'd6;
    localparam [2:0] OP_STATUS     = 3'd7;

    reg [9:0] presc_cnt;
    reg [1:0] ext_sync;
    wire ext_rise = ext_sync[0] && !ext_sync[1];

    // Timer 0 state (12-bit)
    reg [11:0] t0_cnt, t0_cc, t0_top;
    reg [2:0]  t0_presc;
    reg        t0_en, t0_auto_reload, t0_counter_mode, t0_pwm_pol;
    reg        t0_cmp_sticky, t0_per_sticky;

    // Timer 1 state (10-bit)
    reg [9:0]  t1_cnt, t1_cc, t1_top;
    reg [2:0]  t1_presc;
    reg        t1_en, t1_auto_reload, t1_counter_mode, t1_pwm_pol;
    reg        t1_cmp_sticky, t1_per_sticky;

    function automatic presc_match;
        input [2:0] p;
        input [9:0] c;
        begin
            case (p)
                3'd0:    presc_match = 1'b1;
                3'd1:    presc_match = c[0];
                3'd2:    presc_match = &c[1:0];
                3'd3:    presc_match = &c[2:0];
                3'd4:    presc_match = &c[3:0];
                3'd5:    presc_match = &c[5:0];
                3'd6:    presc_match = &c[7:0];
                default: presc_match = &c[9:0];
            endcase
        end
    endfunction

    wire t0_presc_tick = presc_match(t0_presc, presc_cnt);
    wire t1_presc_tick = presc_match(t1_presc, presc_cnt);

    assign pwm0_out       = t0_en ? ((t0_cnt < t0_cc) ^ t0_pwm_pol) : t0_pwm_pol;
    assign pwm1_out       = t1_en ? ((t1_cnt < t1_cc) ^ t1_pwm_pol) : t1_pwm_pol;
    assign dma_cnt0_rdata = {4'd0, t0_cnt};
    assign dma_cnt1_rdata = {6'd0, t1_cnt};
    wire _unused_dma_cc   = &{1'b0, dma_cc_wdata[15:12]};

    wire sel_t1 = req_op[3];
    wire [2:0] subop = req_op[2:0];
    wire [7:0] status_byte = {t1_per_sticky, t0_per_sticky, t1_cmp_sticky, t0_cmp_sticky,
                              pwm1_out, pwm0_out, t1_en, t0_en};

    always @(*) begin
        case (subop)
            OP_CFG:        resp_data16 = {8'd0, req_data};
            OP_SET_CC_LO:  resp_data16 = {sel_t1 ? {6'd0, t1_cc[9:8]}  : {4'd0, t0_cc[11:8]},  req_data};
            OP_SET_CC_HI:  resp_data16 = {sel_t1 ? {6'd0, req_data[1:0]} : {4'd0, req_data[3:0]}, sel_t1 ? t1_cc[7:0]  : t0_cc[7:0]};
            OP_SET_TOP_LO: resp_data16 = {sel_t1 ? {6'd0, t1_top[9:8]} : {4'd0, t0_top[11:8]}, req_data};
            OP_SET_TOP_HI: resp_data16 = {sel_t1 ? {6'd0, req_data[1:0]} : {4'd0, req_data[3:0]}, sel_t1 ? t1_top[7:0] : t0_top[7:0]};
            OP_READ_CNT,
            OP_CTRL:       resp_data16 = sel_t1 ? {6'd0, t1_cnt} : {4'd0, t0_cnt};
            OP_STATUS:     resp_data16 = {8'd0, status_byte};
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            presc_cnt       <= 10'd0;
            ext_sync        <= 2'b00;

            t0_cnt          <= 12'd0;
            t0_cc           <= 12'd0;
            t0_top          <= 12'hFFF;
            t0_presc        <= 3'd0;
            t0_en           <= 1'b0;
            t0_auto_reload  <= 1'b1;
            t0_counter_mode <= 1'b0;
            t0_pwm_pol      <= 1'b0;
            t0_cmp_sticky   <= 1'b0;
            t0_per_sticky   <= 1'b0;

            t1_cnt          <= 10'd0;
            t1_cc           <= 10'd0;
            t1_top          <= 10'h3FF;
            t1_presc        <= 3'd0;
            t1_en           <= 1'b0;
            t1_auto_reload  <= 1'b1;
            t1_counter_mode <= 1'b0;
            t1_pwm_pol      <= 1'b0;
            t1_cmp_sticky   <= 1'b0;
            t1_per_sticky   <= 1'b0;

            evt_t0_cmp      <= 1'b0;
            evt_t0_period   <= 1'b0;
            evt_t1_cmp      <= 1'b0;
            evt_t1_period   <= 1'b0;
        end else begin : main_blk
            reg t0_tick_eff, t1_tick_eff;
            reg t0_clr_now, t1_clr_now;

            presc_cnt     <= presc_cnt + 10'd1;
            ext_sync      <= {ext_sync[0], ext_cap_in};
            evt_t0_cmp    <= 1'b0;
            evt_t0_period <= 1'b0;
            evt_t1_cmp    <= 1'b0;
            evt_t1_period <= 1'b0;

            t0_clr_now = 1'b0;
            t1_clr_now = 1'b0;

            // EasyDMA CC updates (applied cleanly at period boundary)
            if (dma_cc0_we) t0_cc <= dma_cc_wdata[11:0];
            if (dma_cc1_we) t1_cc <= dma_cc_wdata[9:0];

            // PPI tasks
            if (t0_task_start_clr) begin
                t0_en      <= 1'b1;
                t0_clr_now = 1'b1;
            end
            if (t0_task_stop) begin
                t0_en <= 1'b0;
            end
            if (t1_task_start_clr) begin
                t1_en      <= 1'b1;
                t1_clr_now = 1'b1;
            end

            // Register accesses
            if (req_valid) begin
                if (!sel_t1) begin
                    case (subop)
                        OP_CFG: begin
                            t0_presc        <= req_data[6:4];
                            t0_pwm_pol      <= req_data[3];
                            t0_counter_mode <= req_data[2];
                            t0_auto_reload  <= req_data[1];
                            t0_en           <= req_data[0];
                        end
                        OP_SET_CC_LO:  t0_cc[7:0]   <= req_data;
                        OP_SET_CC_HI:  t0_cc[11:8]  <= req_data[3:0];
                        OP_SET_TOP_LO: t0_top[7:0]  <= req_data;
                        OP_SET_TOP_HI: t0_top[11:8] <= req_data[3:0];
                        OP_CTRL: begin
                            if (req_data[0]) begin t0_en <= 1'b1; t0_clr_now = 1'b1; end
                            if (req_data[1]) t0_en <= 1'b0;
                            if (req_data[2]) begin t0_cmp_sticky <= 1'b0; t0_per_sticky <= 1'b0; end
                        end
                        OP_STATUS: begin
                            if (req_data[0]) begin t0_cmp_sticky <= 1'b0; t0_per_sticky <= 1'b0; end
                        end
                        default: begin end
                    endcase
                end else begin
                    case (subop)
                        OP_CFG: begin
                            t1_presc        <= req_data[6:4];
                            t1_pwm_pol      <= req_data[3];
                            t1_counter_mode <= req_data[2];
                            t1_auto_reload  <= req_data[1];
                            t1_en           <= req_data[0];
                        end
                        OP_SET_CC_LO:  t1_cc[7:0]  <= req_data;
                        OP_SET_CC_HI:  t1_cc[9:8]  <= req_data[1:0];
                        OP_SET_TOP_LO: t1_top[7:0] <= req_data;
                        OP_SET_TOP_HI: t1_top[9:8] <= req_data[1:0];
                        OP_CTRL: begin
                            if (req_data[0]) begin t1_en <= 1'b1; t1_clr_now = 1'b1; end
                            if (req_data[1]) t1_en <= 1'b0;
                            if (req_data[2]) begin t1_cmp_sticky <= 1'b0; t1_per_sticky <= 1'b0; end
                        end
                        OP_STATUS: begin
                            if (req_data[0]) begin t1_cmp_sticky <= 1'b0; t1_per_sticky <= 1'b0; end
                        end
                        default: begin end
                    endcase
                end
            end

            // Timer 0 counting & compare/period events
            t0_tick_eff = t0_en && (t0_counter_mode ? ext_rise : t0_presc_tick);
            if (t0_clr_now) begin
                t0_cnt <= 12'd0;
            end else if (t0_tick_eff) begin
                if (t0_cnt == t0_cc) begin
                    evt_t0_cmp    <= 1'b1;
                    t0_cmp_sticky <= 1'b1;
                end
                if (t0_cnt >= t0_top) begin
                    evt_t0_period <= 1'b1;
                    t0_per_sticky <= 1'b1;
                    if (t0_auto_reload) t0_cnt <= 12'd0;
                    else                t0_en  <= 1'b0;
                end else begin
                    t0_cnt <= t0_cnt + 12'd1;
                end
            end

            // Timer 1 counting & compare/period events
            t1_tick_eff = t1_en && (t1_counter_mode ? (t1_task_count || ext_rise) : t1_presc_tick);
            if (t1_clr_now) begin
                t1_cnt <= 10'd0;
            end else if (t1_tick_eff) begin
                if (t1_cnt == t1_cc) begin
                    evt_t1_cmp    <= 1'b1;
                    t1_cmp_sticky <= 1'b1;
                end
                if (t1_cnt >= t1_top) begin
                    evt_t1_period <= 1'b1;
                    t1_per_sticky <= 1'b1;
                    if (t1_auto_reload) t1_cnt <= 10'd0;
                    else                t1_en  <= 1'b0;
                end else begin
                    t1_cnt <= t1_cnt + 10'd1;
                end
            end
        end
    end

endmodule
