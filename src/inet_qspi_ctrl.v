/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shared 1-4-4 QSPI Flash (CS0_n) + Dual PSRAM (CS1_n/CS2_n) controller.
 * - 0xEB Quad Read  (Flash & PSRAM): 8 cmd (IO0) + 6 addr (IO[3:0]) + 6 wait + LE quad words
 *   - For Flash (dev==0), first 2 wait clocks drive mode byte 0x00, next 4 Hi-Z.
 *   - For PSRAM (dev!=0), all 6 wait clocks are Hi-Z.
 * - 0x38 Quad Write (PSRAM): 8 cmd (IO0) + 6 addr (IO[3:0]) + 0 wait + LE quad words
 * - Supports 1..32 word streaming bursts (cmd_words = words - 1) and
 *   adjustable qspi_lat[1:0] read capture delay for Tiny Tapeout I/O mux timing.
 */

`default_nettype none

module inet_qspi_ctrl (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  qspi_lat,

    // Command interface
    input  wire        cmd_valid,
    output wire        cmd_ready,
    input  wire [1:0]  cmd_dev,     // 2'd0=Flash CS0, 2'd1=PSRAM A CS1, 2'd2=PSRAM B CS2
    input  wire        cmd_write,   // 0=0xEB read, 1=0x38 write
    input  wire [23:0] cmd_addr,
    input  wire [4:0]  cmd_words,   // burst length - 1 (0 => 1 32-bit word)

    // Write data stream (32-bit little-endian words)
    input  wire [31:0] wdata,
    input  wire        wdata_valid,
    output reg         wdata_ready,

    // Read data stream (32-bit little-endian words)
    output reg  [31:0] rdata,
    output reg         rdata_valid,
    input  wire        rdata_ready,

    output wire        busy,

    // QSPI physical pins
    output reg         qspi_cs0_n,
    output reg         qspi_cs1_n,
    output reg         qspi_cs2_n,
    output reg         qspi_sck,
    output reg  [3:0]  qspi_io_out,
    output reg  [3:0]  qspi_io_oe,
    input  wire [3:0]  qspi_io_in
);

    localparam [3:0] ST_IDLE    = 4'd0;
    localparam [3:0] ST_CMD     = 4'd1;
    localparam [3:0] ST_ADDR    = 4'd2;
    localparam [3:0] ST_WAIT    = 4'd3;
    localparam [3:0] ST_RDATA   = 4'd4;
    localparam [3:0] ST_RWAIT   = 4'd5;
    localparam [3:0] ST_WLOAD   = 4'd6;
    localparam [3:0] ST_WDATA   = 4'd7;
    localparam [3:0] ST_CS_HIGH = 4'd8;

    reg [3:0]  state;
    reg [1:0]  dev_r;
    reg        is_write_r;
    reg [23:0] addr_r;
    reg [4:0]  words_rem;
    reg [7:0]  cmd_shift;
    reg [2:0]  step_cnt;
    reg [1:0]  lat_cnt;
    reg [31:0] shift_buf;

    assign busy      = (state != ST_IDLE);
    assign cmd_ready = (state == ST_IDLE);

    // Select nibble for little-endian 32-bit word in QSPI byte-serial order:
    // byte0 [7:4],[3:0], byte1 [15:12],[11:8], byte2 [23:20],[19:16], byte3 [31:28],[27:24]
    function [3:0] word_nibble;
        input [31:0] w;
        input [2:0]  idx;
        begin
            case (idx)
                3'd0: word_nibble = w[7:4];
                3'd1: word_nibble = w[3:0];
                3'd2: word_nibble = w[15:12];
                3'd3: word_nibble = w[11:8];
                3'd4: word_nibble = w[23:20];
                3'd5: word_nibble = w[19:16];
                3'd6: word_nibble = w[31:28];
                3'd7: word_nibble = w[27:24];
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            dev_r       <= 2'd0;
            is_write_r  <= 1'b0;
            addr_r      <= 24'd0;
            words_rem   <= 5'd0;
            cmd_shift   <= 8'd0;
            step_cnt    <= 3'd0;
            lat_cnt     <= 2'd0;
            shift_buf   <= 32'd0;
            wdata_ready <= 1'b0;
            rdata       <= 32'd0;
            rdata_valid <= 1'b0;
            qspi_cs0_n  <= 1'b1;
            qspi_cs1_n  <= 1'b1;
            qspi_cs2_n  <= 1'b1;
            qspi_sck    <= 1'b0;
            qspi_io_out <= 4'h0;
            qspi_io_oe  <= 4'h0;
        end else begin
            wdata_ready <= 1'b0;
            if (rdata_valid && rdata_ready) begin
                rdata_valid <= 1'b0;
            end

            case (state)
                ST_IDLE: begin
                    qspi_cs0_n <= 1'b1;
                    qspi_cs1_n <= 1'b1;
                    qspi_cs2_n <= 1'b1;
                    qspi_sck   <= 1'b0;
                    qspi_io_oe <= 4'h0;
                    if (cmd_valid) begin : cmd_accept
                        reg [7:0] opcode;
                        opcode = cmd_write ? 8'h38 : 8'hEB;
                        dev_r      <= cmd_dev;
                        is_write_r <= cmd_write;
                        addr_r     <= cmd_addr;
                        words_rem  <= cmd_words;
                        cmd_shift  <= {opcode[6:0], 1'b0};
                        step_cnt   <= 3'd7;
                        qspi_cs0_n <= (cmd_dev != 2'd0);
                        qspi_cs1_n <= (cmd_dev != 2'd1);
                        qspi_cs2_n <= (cmd_dev != 2'd2);
                        qspi_io_oe <= 4'b0001;
                        qspi_io_out <= {3'b000, opcode[7]};
                        qspi_sck   <= 1'b0;
                        state      <= ST_CMD;
                    end
                end

                // Send 8-bit command on IO0 (1-bit SPI)
                ST_CMD: begin
                    if (!qspi_sck) begin
                        qspi_sck <= 1'b1;
                    end else begin
                        qspi_sck <= 1'b0;
                        if (step_cnt == 3'd0) begin
                            qspi_io_oe  <= 4'b1111;
                            qspi_io_out <= addr_r[23:20];
                            addr_r      <= {addr_r[19:0], 4'h0};
                            step_cnt    <= 3'd5;
                            state       <= ST_ADDR;
                        end else begin
                            qspi_io_out <= {3'b000, cmd_shift[7]};
                            cmd_shift   <= {cmd_shift[6:0], 1'b0};
                            step_cnt    <= step_cnt - 3'd1;
                        end
                    end
                end

                // Send 24-bit address on IO[3:0] (6 nibbles)
                ST_ADDR: begin
                    if (!qspi_sck) begin
                        qspi_sck <= 1'b1;
                    end else begin
                        qspi_sck <= 1'b0;
                        if (step_cnt == 3'd0) begin
                            if (is_write_r) begin
                                qspi_io_oe <= 4'b1111;
                                state      <= ST_WLOAD;
                            end else begin
                                // 6 wait clocks: Flash drives mode 0x00 for first 2 clocks;
                                // PSRAM keeps Hi-Z for all 6 wait clocks.
                                qspi_io_oe  <= (dev_r == 2'd0) ? 4'b1111 : 4'b0000;
                                qspi_io_out <= 4'h0;
                                step_cnt    <= 3'd5;
                                state       <= ST_WAIT;
                            end
                        end else begin
                            qspi_io_out <= addr_r[23:20];
                            addr_r      <= {addr_r[19:0], 4'h0};
                            step_cnt    <= step_cnt - 3'd1;
                        end
                    end
                end

                // 6 wait/mode+dummy clocks for 0xEB
                ST_WAIT: begin
                    if (!qspi_sck) begin
                        qspi_sck <= 1'b1;
                    end else begin
                        qspi_sck <= 1'b0;
                        if (step_cnt == 3'd0) begin
                            qspi_io_oe <= 4'b0000;
                            step_cnt   <= 3'd0;
                            lat_cnt    <= qspi_lat;
                            state      <= ST_RDATA;
                        end else begin
                            // After 2 mode nibbles (step_cnt 5,4 -> next is 3), release IO for Flash
                            if (dev_r == 2'd0 && step_cnt <= 3'd4) begin
                                qspi_io_oe <= 4'b0000;
                            end
                            step_cnt <= step_cnt - 3'd1;
                        end
                    end
                end

                // Read 8 nibbles per 32-bit LE word
                ST_RDATA: begin
                    if (!qspi_sck) begin
                        qspi_sck <= 1'b1;
                        lat_cnt  <= qspi_lat;
                    end else if (lat_cnt != 2'd0) begin
                        lat_cnt <= lat_cnt - 2'd1;
                    end else begin : r_cap
                        reg [31:0] next_word;
                        next_word = shift_buf;
                        case (step_cnt)
                            3'd0: next_word[7:4]   = qspi_io_in;
                            3'd1: next_word[3:0]   = qspi_io_in;
                            3'd2: next_word[15:12] = qspi_io_in;
                            3'd3: next_word[11:8]  = qspi_io_in;
                            3'd4: next_word[23:20] = qspi_io_in;
                            3'd5: next_word[19:16] = qspi_io_in;
                            3'd6: next_word[31:28] = qspi_io_in;
                            3'd7: next_word[27:24] = qspi_io_in;
                        endcase
                        shift_buf <= next_word;
                        qspi_sck  <= 1'b0;

                        if (step_cnt == 3'd7) begin
                            rdata       <= next_word;
                            rdata_valid <= 1'b1;
                            if (words_rem == 5'd0) begin
                                state <= ST_CS_HIGH;
                            end else begin
                                words_rem <= words_rem - 5'd1;
                                step_cnt  <= 3'd0;
                                state     <= ST_RWAIT;
                            end
                        end else begin
                            step_cnt <= step_cnt + 3'd1;
                        end
                    end
                end

                // Pause SCK low if consumer hasn't drained previous word yet
                ST_RWAIT: begin
                    if (!rdata_valid || rdata_ready) begin
                        lat_cnt <= qspi_lat;
                        state   <= ST_RDATA;
                    end
                end

                // Accept next 32-bit write word
                ST_WLOAD: begin
                    if (wdata_valid) begin
                        shift_buf   <= wdata;
                        wdata_ready <= 1'b1;
                        qspi_io_oe  <= 4'b1111;
                        qspi_io_out <= word_nibble(wdata, 3'd0);
                        step_cnt    <= 3'd0;
                        state       <= ST_WDATA;
                    end
                end

                // Stream 8 nibbles of current write word
                ST_WDATA: begin
                    if (!qspi_sck) begin
                        qspi_sck <= 1'b1;
                    end else begin
                        qspi_sck <= 1'b0;
                        if (step_cnt == 3'd7) begin
                            if (words_rem == 5'd0) begin
                                qspi_io_oe <= 4'b0000;
                                state      <= ST_CS_HIGH;
                            end else begin
                                words_rem <= words_rem - 5'd1;
                                state     <= ST_WLOAD;
                            end
                        end else begin : w_next
                            reg [2:0] nxt;
                            nxt = step_cnt + 3'd1;
                            step_cnt    <= nxt;
                            qspi_io_out <= word_nibble(shift_buf, nxt);
                        end
                    end
                end

                ST_CS_HIGH: begin
                    qspi_cs0_n <= 1'b1;
                    qspi_cs1_n <= 1'b1;
                    qspi_cs2_n <= 1'b1;
                    qspi_sck   <= 1'b0;
                    qspi_io_oe <= 4'h0;
                    state      <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
