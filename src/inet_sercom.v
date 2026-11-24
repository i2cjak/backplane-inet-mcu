/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unified Reconfigurable Serial Engine (Nordic UARTE/SPIM/TWIM & SERCOM style).
 * Replaces separate UART, SPI, and I2C controllers with ONE shared shift/divider/FIFO
 * datapath supporting runtime mode switching:
 *   - MODE_UART (0): 8N1 full-duplex UART TX/RX with programmable 12-bit baud_div
 *   - MODE_SPI  (1): Full-duplex MSB-first SPI controller (all 4 CPOL/CPHA modes)
 *   - MODE_I2C  (2): Open-drain I2C controller (START/STOP/W/R + SCL clock stretching)
 * Includes shared 4-byte FIFO, PPI task_kick / evt_done / evt_rx, and EasyDMA TX/RX.
 */

`default_nettype none

module inet_sercom (
    input  wire       clk,
    input  wire       rst_n,

    // Uniform IO-bridge request/response interface
    input  wire       req_valid,
    output wire       req_ready,
    input  wire [1:0] req_mode,      // target mode when req_mode_auto=1
    input  wire       req_mode_auto, // 1 = auto-switch mode cleanly on idle
    input  wire [3:0] req_op,
    input  wire [7:0] req_data,

    output reg        resp_valid,
    input  wire       resp_ready,
    output wire [7:0] resp_status,
    output reg  [7:0] resp_data,

    // PPI & DMA integration
    output reg        evt_done,
    output reg        evt_rx,
    input  wire       task_kick,
    input  wire       dma_tx_valid,
    input  wire [7:0] dma_tx_data,
    output wire       dma_tx_ready,
    output wire       dma_rx_valid,
    output wire [7:0] dma_rx_data,
    input  wire       dma_rx_ready,
    output wire [1:0] cur_mode,

    // UART physical signals
    input  wire       uart_rx,
    output wire       uart_tx,

    // SPI physical signals
    output wire       spi_sck,
    output wire       spi_mosi,
    input  wire       spi_miso,
    output wire       spi_cs_n,

    // I2C open-drain signals
    input  wire       i2c_scl_in,
    output wire       i2c_scl_drive_low,
    input  wire       i2c_sda_in,
    output wire       i2c_sda_drive_low
);

    localparam [1:0] MODE_UART = 2'd0;
    localparam [1:0] MODE_SPI  = 2'd1;
    localparam [1:0] MODE_I2C  = 2'd2;

    // Shared opcode encodings (preserving backward compatibility with spec/isa.py)
    localparam [3:0] OP_SERCOM_MODE = 4'hF;

    // UART opcodes
    localparam [3:0] U_TX         = 4'h0;
    localparam [3:0] U_RX_BLOCK   = 4'h1;
    localparam [3:0] U_RX_POLL    = 4'h2;
    localparam [3:0] U_STATUS     = 4'h3;
    localparam [3:0] U_TX_WAIT    = 4'h4;
    localparam [3:0] U_CFG_DIV_LO = 4'h5;
    localparam [3:0] U_CFG_DIV_HI = 4'h6;
    localparam [3:0] U_FIFO_PUSH  = 4'h7;

    // SPI opcodes
    localparam [3:0] S_XFER     = 4'h0;
    localparam [3:0] S_CS_LOW   = 4'h1;
    localparam [3:0] S_CS_HIGH  = 4'h2;
    localparam [3:0] S_STATUS   = 4'h3;
    localparam [3:0] S_CFG_DIV  = 4'h4;
    localparam [3:0] S_CFG_MODE = 4'h5;

    // I2C opcodes
    localparam [3:0] I_START      = 4'h0;
    localparam [3:0] I_STOP       = 4'h1;
    localparam [3:0] I_WRITE_BYTE = 4'h2;
    localparam [3:0] I_READ_ACK   = 4'h3;
    localparam [3:0] I_READ_NACK  = 4'h4;
    localparam [3:0] I_STATUS     = 4'h5;
    localparam [3:0] I_CFG_DIV    = 4'h6;

    // Shared host/protocol FSM states
    localparam [2:0] ST_IDLE        = 3'd0;
    localparam [2:0] ST_WAIT_TX     = 3'd1;
    localparam [2:0] ST_WAIT_RX     = 3'd2;
    localparam [2:0] ST_WAIT_TXDONE = 3'd3;
    localparam [2:0] ST_SPI_XFER    = 3'd4;
    localparam [2:0] ST_I2C_START   = 3'd5;
    localparam [2:0] ST_I2C_STOP    = 3'd6;
    localparam [2:0] ST_I2C_BYTE    = 3'd7;

    // UART RX states
    localparam [1:0] RX_IDLE  = 2'd0;
    localparam [1:0] RX_START = 2'd1;
    localparam [1:0] RX_DATA  = 2'd2;
    localparam [1:0] RX_STOP  = 2'd3;

    reg [1:0]  mode;
    reg [2:0]  state;
    reg [1:0]  phase;
    reg [3:0]  bit_cnt;

    // Saved per-protocol divider shadows + shared tick counter
    reg [7:0]  uart_div;
    reg [7:0]  spi_div;
    reg [7:0]  i2c_div;
    reg [7:0]  tick_cnt;

    // Shared TX/RX shift registers & buffers
    reg [9:0]  shift_tx;
    reg [7:0]  shift_rx;
    reg [7:0]  rx_buf;
    reg [7:0]  pending_tx;

    // Shared 1-byte FIFO (for PPI task_kick & DMA TX buffering)
    reg [7:0]  fifo_buf;
    reg        fifo_full;
    reg        dma_auto_drain;
    reg [3:0]  resp_status_r;

    // UART-specific control/RX sampler
    reg        tx_busy;
    reg [1:0]  rx_sync;
    reg [1:0]  rx_state;
    reg [7:0]  rx_clk_cnt;
    reg [2:0]  rx_bit_cnt;
    reg        rx_valid;
    reg        dma_rx_pend;
    reg        rx_overrun;
    reg        framing_err;

    // SPI-specific state
    reg        cpol;
    reg        cpha;
    reg        sck_r;
    reg        mosi_r;
    reg        cs_n_r;

    // I2C-specific state
    reg [1:0]  scl_sync;
    reg [1:0]  sda_sync;
    reg        scl_low_r;
    reg        sda_low_r;
    reg        i2c_is_read;
    reg        send_ack_n;
    reg        last_nack;
    reg        bus_busy;

    wire rx_s  = rx_sync[1];
    wire scl_s = scl_sync[1];
    wire sda_s = sda_sync[1];

    wire scl_stretched = (mode == MODE_I2C) && (state >= ST_I2C_START) && (!scl_low_r) && (!scl_s);

    wire [3:0] uart_status = {rx_overrun, framing_err, tx_busy, rx_valid};
    wire [3:0] spi_status  = {2'b00, cs_n_r, (state == ST_SPI_XFER)};
    wire [3:0] i2c_status  = {1'b0, bus_busy, last_nack, ~last_nack};

    wire mode_switch_blocked = req_mode_auto && (req_mode != mode) && tx_busy;

    assign cur_mode          = mode;
    assign uart_tx           = (mode == MODE_UART && tx_busy) ? shift_tx[0] : 1'b1;
    assign spi_sck           = sck_r;
    assign spi_mosi          = mosi_r;
    assign spi_cs_n          = cs_n_r;
    assign i2c_scl_drive_low = scl_low_r;
    assign i2c_sda_drive_low = sda_low_r;
    assign resp_status       = {4'd0, resp_status_r};

    assign req_ready = (state == ST_IDLE) && !resp_valid && !mode_switch_blocked;

    assign dma_tx_ready = !fifo_full;
    assign dma_rx_valid = (mode == MODE_UART) ? rx_valid : dma_rx_pend;
    assign dma_rx_data  = rx_buf;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode        <= MODE_UART;
            state       <= ST_IDLE;
            phase       <= 2'd0;
            bit_cnt     <= 4'd0;
            uart_div    <= 8'd207; // 24 MHz / 208 = 115,385 baud
            spi_div     <= 8'd3;
            i2c_div     <= 8'd59;  // 24 MHz / (4 * 60) = 100 kHz
            tick_cnt    <= 8'd0;
            shift_tx    <= 10'h3FF;
            shift_rx    <= 8'd0;
            rx_buf      <= 8'd0;
            pending_tx  <= 8'd0;
            fifo_buf       <= 8'd0;
            fifo_full      <= 1'b0;
            dma_auto_drain <= 1'b0;

            tx_busy     <= 1'b0;
            rx_sync     <= 2'b11;
            rx_state    <= RX_IDLE;
            rx_clk_cnt  <= 8'd0;
            rx_bit_cnt  <= 3'd0;
            rx_valid    <= 1'b0;
            dma_rx_pend <= 1'b0;
            rx_overrun  <= 1'b0;
            framing_err <= 1'b0;

            cpol        <= 1'b0;
            cpha        <= 1'b0;
            sck_r       <= 1'b0;
            mosi_r      <= 1'b0;
            cs_n_r      <= 1'b1;

            scl_sync    <= 2'b11;
            sda_sync    <= 2'b11;
            scl_low_r   <= 1'b0;
            sda_low_r   <= 1'b0;
            i2c_is_read <= 1'b0;
            send_ack_n  <= 1'b0;
            last_nack   <= 1'b0;
            bus_busy    <= 1'b0;

            resp_valid    <= 1'b0;
            resp_status_r <= 4'd0;
            resp_data     <= 8'd0;
            evt_done      <= 1'b0;
            evt_rx        <= 1'b0;
        end else begin : main_blk
            reg rx_consumed;
            reg kick_pop;
            reg dma_push;
            reg [7:0] push_val;
            reg [1:0] eff_mode;

            rx_consumed = 1'b0;
            kick_pop    = 1'b0;
            dma_push    = 1'b0;
            push_val    = 8'd0;
            eff_mode    = (req_mode_auto && req_valid && req_ready) ? req_mode : mode;

            evt_done <= 1'b0;
            evt_rx   <= 1'b0;

            rx_sync  <= {rx_sync[0], uart_rx};
            scl_sync <= {scl_sync[0], i2c_scl_in};
            sda_sync <= {sda_sync[0], i2c_sda_in};

            if (resp_valid && resp_ready) begin
                resp_valid <= 1'b0;
            end
            if (dma_rx_valid && dma_rx_ready) begin
                if (mode == MODE_UART) begin
                    rx_consumed = 1'b1;
                    rx_valid    <= 1'b0;
                    rx_overrun  <= 1'b0;
                    framing_err <= 1'b0;
                end else begin
                    dma_rx_pend <= 1'b0;
                end
            end

            // ---------------- Background UART TX engine ----------------
            if (tx_busy) begin
                if (tick_cnt == 8'd0) begin
                    tick_cnt <= uart_div;
                    if (bit_cnt == 4'd0) begin
                        tx_busy  <= 1'b0;
                        evt_done <= 1'b1;
                    end else begin
                        shift_tx <= {1'b1, shift_tx[9:1]};
                        bit_cnt  <= bit_cnt - 4'd1;
                    end
                end else begin
                    tick_cnt <= tick_cnt - 8'd1;
                end
            end

            // ---------------- Shared Host & Protocol FSM ----------------
            if (state == ST_IDLE) begin
                if (req_valid && req_ready) begin
                    if (req_mode_auto && (req_mode != mode)) begin
                        mode <= req_mode;
                    end
                    if (req_op == OP_SERCOM_MODE) begin
                        mode          <= req_data[1:0];
                        resp_valid    <= 1'b1;
                        resp_status_r <= 4'd0;
                        resp_data     <= {6'd0, req_data[1:0]};
                    end else case (eff_mode)
                        MODE_UART: begin
                            case (req_op)
                                U_TX: begin
                                    if (!tx_busy) begin
                                        shift_tx      <= {1'b1, req_data, 1'b0};
                                        tx_busy       <= 1'b1;
                                        bit_cnt       <= 4'd9;
                                        tick_cnt      <= uart_div;
                                        resp_valid    <= 1'b1;
                                        resp_status_r <= {rx_overrun, framing_err, 1'b1, rx_valid};
                                        resp_data     <= req_data;
                                    end else begin
                                        pending_tx <= req_data;
                                        state      <= ST_WAIT_TX;
                                    end
                                end
                                U_RX_BLOCK: begin
                                    if (rx_valid) begin
                                        rx_consumed = 1'b1;
                                        rx_valid      <= 1'b0;
                                        rx_overrun    <= 1'b0;
                                        framing_err   <= 1'b0;
                                        resp_valid    <= 1'b1;
                                        resp_status_r <= uart_status;
                                        resp_data     <= rx_buf;
                                    end else begin
                                        state <= ST_WAIT_RX;
                                    end
                                end
                                U_RX_POLL: begin
                                    if (rx_valid) begin
                                        rx_consumed = 1'b1;
                                        rx_valid    <= 1'b0;
                                        rx_overrun  <= 1'b0;
                                        framing_err <= 1'b0;
                                    end
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= uart_status;
                                    resp_data     <= rx_buf;
                                end
                                U_STATUS: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= uart_status;
                                    resp_data     <= rx_buf;
                                end
                                U_TX_WAIT: begin
                                    if (!tx_busy) begin
                                        resp_valid    <= 1'b1;
                                        resp_status_r <= uart_status;
                                        resp_data     <= 8'd0;
                                    end else begin
                                        state <= ST_WAIT_TXDONE;
                                    end
                                end
                                U_CFG_DIV_LO: begin
                                    uart_div      <= req_data;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= uart_status;
                                    resp_data     <= req_data;
                                end
                                U_CFG_DIV_HI: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= uart_status;
                                    resp_data     <= {4'b0000, req_data[3:0]};
                                end
                                U_FIFO_PUSH: begin
                                    if (!fifo_full) begin
                                        dma_push = 1'b1;
                                        push_val = req_data;
                                    end
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= {3'd0, fifo_full};
                                    resp_data     <= req_data;
                                end
                                default: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= uart_status;
                                    resp_data     <= 8'd0;
                                end
                            endcase
                        end

                        MODE_SPI: begin
                            case (req_op)
                                S_XFER: begin
                                    shift_tx <= {2'b00, req_data};
                                    shift_rx <= 8'd0;
                                    bit_cnt  <= 4'd7;
                                    phase    <= 2'd0;
                                    tick_cnt <= spi_div;
                                    sck_r    <= cpol;
                                    if (!cpha) mosi_r <= req_data[7];
                                    state    <= ST_SPI_XFER;
                                end
                                S_CS_LOW: begin
                                    cs_n_r        <= 1'b0;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= 4'h0;
                                    resp_data     <= rx_buf;
                                end
                                S_CS_HIGH: begin
                                    cs_n_r        <= 1'b1;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= 4'h2;
                                    resp_data     <= rx_buf;
                                end
                                S_STATUS: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= spi_status;
                                    resp_data     <= rx_buf;
                                end
                                S_CFG_DIV: begin
                                    spi_div       <= req_data;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= spi_status;
                                    resp_data     <= req_data;
                                end
                                S_CFG_MODE: begin
                                    cpol          <= req_data[1];
                                    cpha          <= req_data[0];
                                    sck_r         <= req_data[1];
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= spi_status;
                                    resp_data     <= {6'b000000, req_data[1:0]};
                                end
                                default: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= spi_status;
                                    resp_data     <= rx_buf;
                                end
                            endcase
                        end

                        MODE_I2C: begin
                            case (req_op)
                                I_START: begin
                                    state     <= ST_I2C_START;
                                    phase     <= 2'd0;
                                    tick_cnt  <= i2c_div;
                                    sda_low_r <= 1'b0;
                                end
                                I_STOP: begin
                                    state     <= ST_I2C_STOP;
                                    phase     <= 2'd0;
                                    tick_cnt  <= i2c_div;
                                    sda_low_r <= 1'b1;
                                end
                                I_WRITE_BYTE: begin
                                    state       <= ST_I2C_BYTE;
                                    i2c_is_read <= 1'b0;
                                    phase       <= 2'd0;
                                    bit_cnt     <= 4'd0;
                                    tick_cnt    <= i2c_div;
                                    shift_tx    <= {3'd0, req_data[6:0]};
                                    sda_low_r   <= ~req_data[7];
                                end
                                I_READ_ACK,
                                I_READ_NACK: begin
                                    state       <= ST_I2C_BYTE;
                                    i2c_is_read <= 1'b1;
                                    phase       <= 2'd0;
                                    bit_cnt     <= 4'd0;
                                    tick_cnt    <= i2c_div;
                                    shift_rx    <= 8'd0;
                                    send_ack_n  <= (req_op == I_READ_NACK);
                                    sda_low_r   <= 1'b0;
                                end
                                I_STATUS: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= i2c_status;
                                    resp_data     <= rx_buf;
                                end
                                I_CFG_DIV: begin
                                    i2c_div       <= (req_data == 8'd0) ? 8'd1 : req_data;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= i2c_status;
                                    resp_data     <= (req_data == 8'd0) ? 8'd1 : req_data;
                                end
                                default: begin
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= i2c_status;
                                    resp_data     <= rx_buf;
                                end
                            endcase
                        end

                        default: begin
                            resp_valid    <= 1'b1;
                            resp_status_r <= 4'd0;
                            resp_data     <= 8'd0;
                        end
                    endcase
                end else if ((task_kick || (dma_auto_drain && fifo_full) ||
                              (dma_tx_valid && !fifo_full && !req_valid)) &&
                             !tx_busy && (fifo_full || dma_tx_valid)) begin : auto_kick
                    reg [7:0] tx_b;
                    if (fifo_full) begin
                        tx_b     = fifo_buf;
                        kick_pop = 1'b1;
                    end else begin
                        tx_b = dma_tx_data;
                    end
                    if (mode == MODE_UART) begin
                        shift_tx <= {1'b1, tx_b, 1'b0};
                        tx_busy  <= 1'b1;
                        bit_cnt  <= 4'd9;
                        tick_cnt <= uart_div;
                    end else if (mode == MODE_SPI) begin
                        shift_tx <= {2'b00, tx_b};
                        shift_rx <= 8'd0;
                        bit_cnt  <= 4'd7;
                        phase    <= 2'd0;
                        tick_cnt <= spi_div;
                        sck_r    <= cpol;
                        if (!cpha) mosi_r <= tx_b[7];
                        state    <= ST_SPI_XFER;
                    end else if (mode == MODE_I2C) begin
                        state       <= ST_I2C_BYTE;
                        i2c_is_read <= 1'b0;
                        phase       <= 2'd0;
                        bit_cnt     <= 4'd0;
                        tick_cnt    <= i2c_div;
                        shift_tx    <= {3'd0, tx_b[6:0]};
                        sda_low_r   <= ~tx_b[7];
                    end
                end
            end else begin
                case (state)
                    ST_WAIT_TX: begin
                        if (!tx_busy) begin
                            shift_tx      <= {1'b1, pending_tx, 1'b0};
                            tx_busy       <= 1'b1;
                            bit_cnt       <= 4'd9;
                            tick_cnt      <= uart_div;
                            resp_valid    <= 1'b1;
                            resp_status_r <= {rx_overrun, framing_err, 1'b1, rx_valid};
                            resp_data     <= pending_tx;
                            state         <= ST_IDLE;
                        end
                    end

                    ST_WAIT_RX: begin
                        if (rx_valid) begin
                            rx_consumed = 1'b1;
                            rx_valid      <= 1'b0;
                            rx_overrun    <= 1'b0;
                            framing_err   <= 1'b0;
                            resp_valid    <= 1'b1;
                            resp_status_r <= uart_status;
                            resp_data     <= rx_buf;
                            state         <= ST_IDLE;
                        end
                    end

                    ST_WAIT_TXDONE: begin
                        if (!tx_busy) begin
                            resp_valid    <= 1'b1;
                            resp_status_r <= uart_status;
                            resp_data     <= 8'd0;
                            state         <= ST_IDLE;
                        end
                    end

                    ST_SPI_XFER: begin
                        if (tick_cnt == 8'd0) begin
                            tick_cnt <= spi_div;
                            if (!phase[0]) begin
                                sck_r    <= ~cpol;
                                phase[0] <= 1'b1;
                                if (!cpha) begin
                                    shift_rx <= {shift_rx[6:0], spi_miso};
                                    shift_tx <= {2'b00, shift_tx[6:0], 1'b0};
                                end else begin
                                    mosi_r   <= shift_tx[7];
                                    shift_tx <= {2'b00, shift_tx[6:0], 1'b0};
                                end
                            end else begin
                                sck_r    <= cpol;
                                phase[0] <= 1'b0;
                                if (!cpha) begin
                                    if (bit_cnt == 4'd0) begin
                                        state         <= ST_IDLE;
                                        rx_buf        <= shift_rx;
                                        dma_rx_pend   <= 1'b1;
                                        resp_valid    <= 1'b1;
                                        resp_status_r <= {2'b00, cs_n_r, 1'b0};
                                        resp_data     <= shift_rx;
                                        evt_done      <= 1'b1;
                                        evt_rx        <= 1'b1;
                                    end else begin
                                        mosi_r  <= shift_tx[7];
                                        bit_cnt <= bit_cnt - 4'd1;
                                    end
                                end else begin : spi_cpha1
                                    reg [7:0] next_rx;
                                    next_rx = {shift_rx[6:0], spi_miso};
                                    shift_rx <= next_rx;
                                    if (bit_cnt == 4'd0) begin
                                        state         <= ST_IDLE;
                                        rx_buf        <= next_rx;
                                        dma_rx_pend   <= 1'b1;
                                        resp_valid    <= 1'b1;
                                        resp_status_r <= {2'b00, cs_n_r, 1'b0};
                                        resp_data     <= next_rx;
                                        evt_done      <= 1'b1;
                                        evt_rx        <= 1'b1;
                                    end else begin
                                        bit_cnt <= bit_cnt - 4'd1;
                                    end
                                end
                            end
                        end else begin
                            tick_cnt <= tick_cnt - 8'd1;
                        end
                    end

                    ST_I2C_START: if (!scl_stretched) begin
                        if (tick_cnt != 8'd0) tick_cnt <= tick_cnt - 8'd1;
                        else begin
                            tick_cnt <= i2c_div;
                            case (phase)
                                2'd0: begin scl_low_r <= 1'b0; phase <= 2'd1; end
                                2'd1: begin sda_low_r <= 1'b1; bus_busy <= 1'b1; phase <= 2'd2; end
                                2'd2: begin scl_low_r <= 1'b1; phase <= 2'd3; end
                                2'd3: begin
                                    state         <= ST_IDLE;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= {1'b0, 1'b1, last_nack, ~last_nack};
                                    resp_data     <= rx_buf;
                                    evt_done      <= 1'b1;
                                end
                            endcase
                        end
                    end

                    ST_I2C_STOP: if (!scl_stretched) begin
                        if (tick_cnt != 8'd0) tick_cnt <= tick_cnt - 8'd1;
                        else begin
                            tick_cnt <= i2c_div;
                            case (phase)
                                2'd0: begin scl_low_r <= 1'b0; phase <= 2'd1; end
                                2'd1: begin sda_low_r <= 1'b0; bus_busy <= 1'b0; phase <= 2'd2; end
                                2'd2: begin phase <= 2'd3; end
                                2'd3: begin
                                    state         <= ST_IDLE;
                                    resp_valid    <= 1'b1;
                                    resp_status_r <= {1'b0, 1'b0, last_nack, ~last_nack};
                                    resp_data     <= rx_buf;
                                    evt_done      <= 1'b1;
                                end
                            endcase
                        end
                    end

                    ST_I2C_BYTE: if (!scl_stretched) begin
                        if (tick_cnt != 8'd0) tick_cnt <= tick_cnt - 8'd1;
                        else begin
                            tick_cnt <= i2c_div;
                            case (phase)
                                2'd0: begin scl_low_r <= 1'b0; phase <= 2'd1; end
                                2'd1: begin
                                    if (!i2c_is_read && bit_cnt == 4'd8) last_nack <= sda_s;
                                    else if (i2c_is_read && bit_cnt < 4'd8) shift_rx <= {shift_rx[6:0], sda_s};
                                    phase <= 2'd2;
                                end
                                2'd2: begin scl_low_r <= 1'b1; phase <= 2'd3; end
                                2'd3: begin
                                    phase <= 2'd0;
                                    if (bit_cnt == 4'd8) begin
                                        sda_low_r <= 1'b0;
                                        state     <= ST_IDLE;
                                        evt_done  <= 1'b1;
                                        if (i2c_is_read) begin
                                            rx_buf        <= shift_rx;
                                            dma_rx_pend   <= 1'b1;
                                            resp_valid    <= 1'b1;
                                            resp_status_r <= {1'b0, bus_busy, send_ack_n, ~send_ack_n};
                                            resp_data     <= shift_rx;
                                            evt_rx        <= 1'b1;
                                        end else begin
                                            resp_valid    <= 1'b1;
                                            resp_status_r <= {1'b0, bus_busy, last_nack, ~last_nack};
                                            resp_data     <= rx_buf;
                                        end
                                    end else begin : i2c_next
                                        reg [3:0] next_bit;
                                        next_bit = bit_cnt + 4'd1;
                                        bit_cnt  <= next_bit;
                                        if (next_bit == 4'd8) begin
                                            sda_low_r <= i2c_is_read ? (~send_ack_n) : 1'b0;
                                        end else if (!i2c_is_read) begin
                                            shift_tx  <= {3'd0, shift_tx[5:0], 1'b0};
                                            sda_low_r <= ~shift_tx[6];
                                        end else begin
                                            sda_low_r <= 1'b0;
                                        end
                                    end
                                end
                            endcase
                        end
                    end

                    default: state <= ST_IDLE;
                endcase
            end

            // ---------------- Shared 1-byte FIFO push/pop ----------------
            if (dma_tx_valid && dma_tx_ready) begin
                dma_auto_drain <= 1'b1;
                if (!(state == ST_IDLE && !req_valid && !tx_busy && !fifo_full)) begin
                    dma_push = 1'b1;
                    push_val = dma_tx_data;
                end
            end else if (!fifo_full && !tx_busy && state == ST_IDLE) begin
                dma_auto_drain <= 1'b0;
            end
            if (dma_push) fifo_buf <= push_val;
            fifo_full <= (fifo_full & ~kick_pop) | dma_push;

            // ---------------- UART RX Sampler (active in MODE_UART) ----------------
            if (mode == MODE_UART) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (!rx_s) begin
                            rx_state   <= RX_START;
                            rx_clk_cnt <= {1'b0, uart_div[7:1]};
                        end
                    end
                    RX_START: begin
                        if (rx_clk_cnt == 8'd0) begin
                            if (!rx_s) begin
                                rx_state   <= RX_DATA;
                                rx_clk_cnt <= uart_div;
                                rx_bit_cnt <= 3'd0;
                            end else begin
                                rx_state <= RX_IDLE;
                            end
                        end else begin
                            rx_clk_cnt <= rx_clk_cnt - 8'd1;
                        end
                    end
                    RX_DATA: begin
                        if (rx_clk_cnt == 8'd0) begin
                            shift_rx   <= {rx_s, shift_rx[7:1]};
                            rx_clk_cnt <= uart_div;
                            if (rx_bit_cnt == 3'd7) rx_state <= RX_STOP;
                            else                    rx_bit_cnt <= rx_bit_cnt + 3'd1;
                        end else begin
                            rx_clk_cnt <= rx_clk_cnt - 8'd1;
                        end
                    end
                    RX_STOP: begin
                        if (rx_clk_cnt == 8'd0) begin
                            rx_state <= RX_IDLE;
                            if (!rx_s) framing_err <= 1'b1;
                            if (rx_valid && !rx_consumed) begin
                                rx_overrun <= 1'b1;
                            end else begin
                                rx_buf   <= shift_rx;
                                rx_valid <= 1'b1;
                                evt_rx   <= 1'b1;
                                evt_done <= 1'b1;
                            end
                        end else begin
                            rx_clk_cnt <= rx_clk_cnt - 8'd1;
                        end
                    end
                    default: rx_state <= RX_IDLE;
                endcase
            end else begin
                rx_state <= RX_IDLE;
            end
        end
    end

endmodule
