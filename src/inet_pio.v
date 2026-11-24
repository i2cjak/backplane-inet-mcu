/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Compact RP2040-Style Programmable I/O (PIO) Engine.
 *   - 16 x 16-bit instruction memory backed by latch RF (inet_latch_rf, BYPASS=0).
 *   - 1 deterministic state machine supporting all 9 RP2040 opcodes:
 *       JMP, WAIT, IN, OUT, PUSH, PULL, MOV, IRQ, SET
 *     with 16-bit OSR/ISR/X/Y, 16-bit clock divider, side-set/delay, and
 *     2-entry 16-bit TX/RX FIFOs wired into EasyDMA + PPI.
 */

`default_nettype none

module inet_pio (
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (DEV_PIO = 8)
    input  wire        req_valid,
    input  wire [3:0]  req_op,
    input  wire [7:0]  req_data,
    output reg  [15:0] resp_data16,

    // DMA TX / RX FIFO streaming
    input  wire        dma_tx_valid,
    input  wire [15:0] dma_tx_data,
    output wire        dma_tx_ready,
    output wire        dma_rx_valid,
    output wire [15:0] dma_rx_data,
    input  wire        dma_rx_ready,

    // PPI integration
    input  wire        task_start,
    output reg         evt_irq,
    output wire        evt_rx_ne,

    // Pin I/O routed through GPIO crossbar
    input  wire [3:0]  pins_in,
    output reg  [3:0]  pins_out,
    output reg  [3:0]  pins_oe
);

    localparam [3:0] OP_CFG       = 4'd0;
    localparam [3:0] OP_SET_ADDR  = 4'd1;
    localparam [3:0] OP_IMEM_LO   = 4'd2;
    localparam [3:0] OP_IMEM_HI   = 4'd3;
    localparam [3:0] OP_SET_WRAP  = 4'd4;
    localparam [3:0] OP_SET_DIV_L = 4'd5;
    localparam [3:0] OP_SET_DIV_H = 4'd6;
    localparam [3:0] OP_TX_LO     = 4'd7;
    localparam [3:0] OP_TX_HI     = 4'd8;
    localparam [3:0] OP_RX_POP    = 4'd9;
    localparam [3:0] OP_STATUS    = 4'd10;
    localparam [3:0] OP_IRQ_CLR   = 4'd11;
    localparam [3:0] OP_READ_REG  = 4'd12;

    reg        sm_en;
    reg        cfg_autopull;
    reg        cfg_autopush;
    reg        cfg_shift_r_out;
    reg        cfg_shift_r_in;
    reg        cfg_sideset_en;

    reg [1:0]  pc, wrap_top, wrap_bot;
    reg [2:0]  delay_cnt;
    reg [15:0] osr, isr;
    reg [7:0]  reg_x, reg_y;
    reg [4:0]  osr_cnt, isr_cnt;
    reg [3:0]  clk_div, div_cnt;
    reg [3:0]  irq_flags;

    // 4x16 IMEM & TX buffer byte write staging (plain resettable DFF array)
    reg [1:0]   wr_addr;
    reg [7:0]   lo_buf;
    reg [15:0]  imem [0:3];
    wire [15:0] instr = imem[pc];

    // 1-entry 16-bit TX & RX buffers (plus OSR/ISR gives 2-word pipeline)
    reg [15:0] tx_buf, rx_buf;
    reg        tx_full, rx_full;
    reg [3:0]  pins_in_s;

    assign dma_tx_ready = !tx_full;
    assign dma_rx_valid = rx_full;
    assign dma_rx_data  = rx_buf;
    assign evt_rx_ne    = rx_full;

    // Compact 4-way {1, 4, 8, 16} shared IN/OUT/MOV source/shift logic
    wire [7:0]  pio_imm8 = instr[7:0];
    wire [4:0]  pio_nb   = (pio_imm8[4:0] == 5'd0) ? 5'd16 : pio_imm8[4:0];
    wire [2:0]  src_sel  = (instr[15:13] == 3'd5)
                         ? ((pio_imm8[2:0] == 3'd3) ? 3'd7 :
                            (pio_imm8[2] ? (pio_imm8[2:0] - 3'd2) : pio_imm8[2:0]))
                         : ((pio_imm8[7:5] == 3'd3) ? 3'd7 : pio_imm8[7:5]);

    reg [15:0] pio_src_v;
    always @(*) begin
        case (src_sel)
            3'd0:    pio_src_v = {12'd0, pins_in_s};
            3'd1:    pio_src_v = {8'd0, reg_x};
            3'd2:    pio_src_v = {8'd0, reg_y};
            3'd3:    pio_src_v = (!tx_full) ? 16'hFFFF : 16'd0;
            3'd4:    pio_src_v = isr;
            3'd5:    pio_src_v = osr;
            default: pio_src_v = 16'd0;
        endcase
    end
    wire [15:0] cur_osr_w = (cfg_autopull && osr_cnt >= 5'd16 && tx_full) ? tx_buf : osr;

    reg [15:0] new_isr_w, out_bits_w, next_osr_w;
    always @(*) begin
        if (pio_nb[4]) begin
            new_isr_w  = pio_src_v;
            out_bits_w = cur_osr_w;
            next_osr_w = 16'd0;
        end else if (pio_nb[3:0] == 4'd1) begin
            new_isr_w  = cfg_shift_r_in  ? {pio_src_v[0], isr[15:1]}   : {isr[14:0], pio_src_v[0]};
            out_bits_w = cfg_shift_r_out ? {15'd0, cur_osr_w[0]}       : {15'd0, cur_osr_w[15]};
            next_osr_w = cfg_shift_r_out ? {1'd0, cur_osr_w[15:1]}     : {cur_osr_w[14:0], 1'd0};
        end else begin // 8 bits
            new_isr_w  = cfg_shift_r_in  ? {pio_src_v[7:0], isr[15:8]} : {isr[7:0], pio_src_v[7:0]};
            out_bits_w = cfg_shift_r_out ? {8'd0, cur_osr_w[7:0]}      : {8'd0, cur_osr_w[15:8]};
            next_osr_w = cfg_shift_r_out ? {8'd0, cur_osr_w[15:8]}     : {cur_osr_w[7:0], 8'd0};
        end
    end
    wire [7:0] xy_dec = ((pio_imm8[7:5] == 3'd2) ? reg_x : reg_y) - 8'd1;

    // Combinational response data (sampled by io_bridge on req_valid cycle)
    always @(*) begin
        case (req_op)
            OP_CFG,
            OP_SET_WRAP,
            OP_IMEM_LO,
            OP_TX_LO:     resp_data16 = {8'd0, req_data};
            OP_SET_ADDR:  resp_data16 = {14'd0, req_data[1:0]};
            OP_IMEM_HI,
            OP_TX_HI:     resp_data16 = {req_data, lo_buf};
            OP_SET_DIV_L: resp_data16 = {8'd0, req_data};
            OP_SET_DIV_H: resp_data16 = {12'd0, clk_div};
            OP_RX_POP:    resp_data16 = rx_buf;
            OP_STATUS:    resp_data16 = {irq_flags, 1'b0, rx_full, 1'b0, tx_full, 2'd0, pc, pins_out};
            OP_IRQ_CLR:   resp_data16 = {12'd0, irq_flags & ~req_data[3:0]};
            OP_READ_REG: begin
                case (req_data[1:0])
                    2'd0: resp_data16 = {8'd0, reg_x};
                    2'd1: resp_data16 = {8'd0, reg_y};
                    2'd2: resp_data16 = osr;
                    2'd3: resp_data16 = isr;
                endcase
            end
            default:      resp_data16 = 16'd0;
        endcase
    end

    integer im_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sm_en           <= 1'b0;
            cfg_autopull    <= 1'b0;
            cfg_autopush    <= 1'b0;
            cfg_shift_r_out <= 1'b1;
            cfg_shift_r_in  <= 1'b0;
            cfg_sideset_en  <= 1'b0;
            pc              <= 2'd0;
            wrap_top        <= 2'd3;
            wrap_bot        <= 2'd0;
            delay_cnt       <= 3'd0;
            osr             <= 16'd0;
            isr             <= 16'd0;
            reg_x           <= 8'd0;
            reg_y           <= 8'd0;
            osr_cnt         <= 5'd16;
            isr_cnt         <= 5'd0;
            clk_div         <= 4'd0;
            div_cnt         <= 4'd0;
            irq_flags       <= 4'd0;
            pins_out        <= 4'd0;
            pins_oe         <= 4'd0;
            pins_in_s       <= 4'd0;

            wr_addr         <= 2'd0;
            lo_buf          <= 8'd0;
            for (im_i = 0; im_i < 4; im_i = im_i + 1) imem[im_i] <= 16'd0;

            tx_buf          <= 16'd0;
            tx_full         <= 1'b0;
            rx_buf          <= 16'd0;
            rx_full         <= 1'b0;

            evt_irq         <= 1'b0;
        end else begin : main_blk
            reg        tx_push_en, tx_pop_en;
            reg [15:0] tx_push_val;
            reg        rx_push_en, rx_pop_en;
            reg [15:0] rx_push_val;
            reg        sm_tick;

            tx_push_en  = 1'b0;
            tx_pop_en   = 1'b0;
            tx_push_val = 16'd0;
            rx_push_en  = 1'b0;
            rx_pop_en   = 1'b0;
            rx_push_val = 16'd0;

            pins_in_s <= pins_in;
            evt_irq   <= 1'b0;

            if (task_start) sm_en <= 1'b1;

            // ---------------- Register Interface ----------------
            if (req_valid) begin
                case (req_op)
                    OP_CFG: begin
                        sm_en           <= req_data[7];
                        cfg_autopull    <= req_data[6];
                        cfg_autopush    <= req_data[5];
                        cfg_shift_r_out <= req_data[4];
                        cfg_shift_r_in  <= req_data[3];
                        cfg_sideset_en  <= req_data[2];
                        if (req_data[1]) begin
                            tx_full <= 1'b0;
                            rx_full <= 1'b0;
                        end
                        if (req_data[0]) begin
                            pc        <= wrap_bot;
                            delay_cnt <= 3'd0;
                            div_cnt   <= 4'd0;
                            osr_cnt   <= 5'd16;
                            isr_cnt   <= 5'd0;
                            isr       <= 16'd0;
                            osr       <= 16'd0;
                        end
                    end
                    OP_SET_ADDR: begin
                        if (req_data[7]) pc <= req_data[1:0];
                        else             wr_addr <= req_data[1:0];
                    end
                    OP_IMEM_LO,
                    OP_TX_LO: begin
                        lo_buf <= req_data;
                    end
                    OP_IMEM_HI: begin
                        imem[wr_addr] <= {req_data, lo_buf};
                        wr_addr       <= wr_addr + 2'd1;
                    end
                    OP_SET_WRAP: begin
                        wrap_top <= req_data[5:4];
                        wrap_bot <= req_data[1:0];
                    end
                    OP_SET_DIV_L: clk_div <= req_data[3:0];
                    OP_SET_DIV_H: begin end
                    OP_TX_HI: begin
                        if (!tx_full) begin
                            tx_push_en  = 1'b1;
                            tx_push_val = {req_data, lo_buf};
                        end
                    end
                    OP_RX_POP: begin
                        if (rx_full) rx_pop_en = 1'b1;
                    end
                    OP_IRQ_CLR: irq_flags <= irq_flags & ~req_data[3:0];
                    default: begin end
                endcase
            end

            // DMA FIFO streams (when host not simultaneously touching same buffer)
            if (dma_tx_valid && dma_tx_ready && !tx_push_en) begin
                tx_push_en  = 1'b1;
                tx_push_val = dma_tx_data;
            end
            if (dma_rx_valid && dma_rx_ready && !rx_pop_en) begin
                rx_pop_en = 1'b1;
            end

            // ---------------- PIO SM Execution ----------------
            sm_tick = 1'b0;
            if (sm_en) begin
                if (div_cnt == 4'd0) begin
                    div_cnt <= clk_div;
                    sm_tick = 1'b1;
                end else begin
                    div_cnt <= div_cnt - 4'd1;
                end
            end

            if (sm_tick) begin
                if (delay_cnt != 3'd0) begin
                    delay_cnt <= delay_cnt - 3'd1;
                end else begin : exec_blk
                    reg [2:0]  op;
                    reg [2:0]  del;
                    reg [7:0]  imm8;
                    reg        stalled;
                    reg        jmp_taken;
                    reg [1:0]  next_pc;
                    reg        wr_x_en, wr_y_en;
                    reg [7:0]  reg_wval;

                    op        = instr[15:13];
                    del       = instr[10:8];
                    imm8      = instr[7:0];
                    stalled   = 1'b0;
                    jmp_taken = 1'b0;
                    next_pc   = (pc == wrap_top) ? wrap_bot : (pc + 2'd1);
                    wr_x_en   = 1'b0;
                    wr_y_en   = 1'b0;
                    reg_wval  = 8'd0;

                    // Apply side-set on instruction execution
                    if (cfg_sideset_en) begin
                        pins_out[1:0] <= instr[12:11];
                    end

                    case (op)
                        3'd0: begin // JMP
                            case (imm8[7:5])
                                3'd0: jmp_taken = 1'b1;
                                3'd1: jmp_taken = (reg_x == 8'd0);
                                3'd2: begin jmp_taken = (reg_x != 8'd0); wr_x_en = 1'b1; reg_wval = xy_dec; end
                                3'd3: jmp_taken = (reg_y == 8'd0);
                                3'd4: begin jmp_taken = (reg_y != 8'd0); wr_y_en = 1'b1; reg_wval = xy_dec; end
                                3'd5: jmp_taken = (reg_x != reg_y);
                                3'd6: jmp_taken = pins_in_s[0];
                                3'd7: jmp_taken = (osr_cnt < 5'd16);
                            endcase
                            if (jmp_taken) next_pc = imm8[1:0];
                        end

                        3'd1: begin : wait_blk // WAIT
                            reg cond_met;
                            cond_met = 1'b0;
                            case (imm8[6:5])
                                2'd0, 2'd1: cond_met = (pins_in_s[imm8[1:0]] == imm8[7]);
                                2'd2:       cond_met = (irq_flags[imm8[1:0]] == imm8[7]);
                                default:    cond_met = 1'b1;
                            endcase
                            if (!cond_met) stalled = 1'b1;
                        end

                        3'd2: begin : in_blk // IN
                            reg [4:0] new_cnt;
                            new_cnt = isr_cnt + pio_nb;
                            if (cfg_autopush && new_cnt >= 5'd16) begin
                                if (rx_full) begin
                                    stalled = 1'b1;
                                end else begin
                                    rx_push_en  = 1'b1;
                                    rx_push_val = new_isr_w;
                                    isr         <= 16'd0;
                                    isr_cnt     <= 5'd0;
                                end
                            end else begin
                                isr     <= new_isr_w;
                                isr_cnt <= (new_cnt > 5'd16) ? 5'd16 : new_cnt;
                            end
                        end

                        3'd3: begin : out_blk // OUT
                            reg [4:0] cur_cnt;
                            cur_cnt = osr_cnt;
                            if (cfg_autopull && cur_cnt >= 5'd16) begin
                                if (!tx_full) begin
                                    stalled = 1'b1;
                                end else begin
                                    tx_pop_en = 1'b1;
                                    cur_cnt   = 5'd0;
                                end
                            end
                            if (!stalled) begin
                                osr      <= next_osr_w;
                                osr_cnt  <= (cur_cnt + pio_nb > 5'd16) ? 5'd16 : (cur_cnt + pio_nb);
                                reg_wval = out_bits_w[7:0];
                                case (pio_imm8[7:5])
                                    3'd0: pins_out <= out_bits_w[3:0];
                                    3'd1: wr_x_en  = 1'b1;
                                    3'd2: wr_y_en  = 1'b1;
                                    3'd4: pins_oe  <= out_bits_w[3:0];
                                    3'd5: next_pc  = out_bits_w[1:0];
                                    3'd6: isr      <= out_bits_w;
                                    default: begin end
                                endcase
                            end
                        end

                        3'd4: begin // PUSH (imm8[7]==0) / PULL (imm8[7]==1)
                            if (!imm8[7]) begin
                                if (imm8[6] && isr_cnt < 5'd16) begin
                                    // if_full not met -> no-op
                                end else if (rx_full) begin
                                    if (imm8[5]) stalled = 1'b1;
                                end else begin
                                    rx_push_en  = 1'b1;
                                    rx_push_val = isr;
                                    isr         <= 16'd0;
                                    isr_cnt     <= 5'd0;
                                end
                            end else begin
                                if (imm8[6] && osr_cnt < 5'd16) begin
                                    // if_empty not met -> no-op
                                end else if (!tx_full) begin
                                    if (imm8[5]) stalled = 1'b1;
                                    else begin osr <= {8'd0, reg_x}; osr_cnt <= 5'd0; end
                                end else begin
                                    osr       <= tx_buf;
                                    tx_pop_en = 1'b1;
                                    osr_cnt   <= 5'd0;
                                end
                            end
                        end

                        3'd5: begin : mov_blk // MOV
                            reg [15:0] mv;
                            mv = (imm8[4:3] == 2'd1) ? ~pio_src_v : pio_src_v;
                            reg_wval = mv[7:0];
                            case (imm8[7:5])
                                3'd0: pins_out <= mv[3:0];
                                3'd1: wr_x_en  = 1'b1;
                                3'd2: wr_y_en  = 1'b1;
                                3'd5: next_pc  = mv[1:0];
                                3'd6: begin isr <= mv; isr_cnt <= 5'd0; end
                                3'd7: begin osr <= mv; osr_cnt <= 5'd0; end
                                default: begin end
                            endcase
                        end

                        3'd6: begin // IRQ
                            if (imm8[6]) begin
                                irq_flags[imm8[1:0]] <= 1'b0;
                            end else begin
                                irq_flags[imm8[1:0]] <= 1'b1;
                                evt_irq              <= 1'b1;
                                if (imm8[5] && irq_flags[imm8[1:0]]) stalled = 1'b1;
                            end
                        end

                        3'd7: begin // SET
                            reg_wval = {3'd0, imm8[4:0]};
                            case (imm8[7:5])
                                3'd0: pins_out <= imm8[3:0];
                                3'd1: wr_x_en  = 1'b1;
                                3'd2: wr_y_en  = 1'b1;
                                3'd4: pins_oe  <= imm8[3:0];
                                default: begin end
                            endcase
                        end
                    endcase

                    if (wr_x_en) reg_x <= reg_wval;
                    if (wr_y_en) reg_y <= reg_wval;

                    if (!stalled) begin
                        pc        <= next_pc;
                        delay_cnt <= del;
                    end
                end
            end

            // ---------------- TX / RX 1-entry buffer updates ----------------
            if (tx_push_en) tx_buf <= tx_push_val;
            tx_full <= (tx_full & ~tx_pop_en) | tx_push_en;

            if (rx_push_en) rx_buf <= rx_push_val;
            rx_full <= (rx_full & ~rx_pop_en) | rx_push_en;
        end
    end

endmodule
