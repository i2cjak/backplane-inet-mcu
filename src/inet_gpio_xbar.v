/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Nordic PSEL-Style Full GPIO Pin-Select Crossbar (non-QSPI, non-SWD pins).
 *   - QSPI (uo[2:0], uio[3:0], uio[7], ui[6], ui[5:4], ui[2]) and SWD (ui[0], uio[4])
 *     remain static/hardwired for clean QSPI timing and uninterrupted debug.
 *   - Routes the remaining 7 output-capable pins (uo[7:3], uio[6:5]) and 5 input-capable
 *     pins (ui[1], ui[3], ui[7], uio[5], uio[6]) across SERCOM, PWM0/1, PIO, Timer,
 *     PPI GPIO edge, STATUS, and raw GPIO.
 */

`default_nettype none

module inet_gpio_xbar (
    input  wire        clk,
    input  wire        rst_n,

    // Register interface (DEV_XBAR = 9)
    input  wire        req_valid,
    input  wire [3:0]  req_op,
    input  wire [7:0]  req_data,
    output reg  [15:0] resp_data16,

    // Non-QSPI/SWD physical pins
    input  wire        pin_ui_1,
    input  wire        pin_ui_3,
    input  wire        pin_ui_7,
    input  wire        pin_uio_5_in,
    input  wire        pin_uio_6_in,

    output reg         pin_uo_3,
    output reg         pin_uo_4,
    output reg         pin_uo_5,
    output reg         pin_uo_6,
    output reg         pin_uo_7,
    output reg         pin_uio_5_out,
    output reg         pin_uio_5_oe,
    output reg         pin_uio_6_out,
    output reg         pin_uio_6_oe,

    // Peripheral signals
    input  wire        uart_tx,
    input  wire        spi_sck,
    input  wire        spi_mosi,
    input  wire        spi_cs_n,
    input  wire        i2c_scl_drive_low,
    input  wire        i2c_sda_drive_low,
    input  wire        pwm0_out,
    input  wire        pwm1_out,
    input  wire [3:0]  pio_pins_out,
    input  wire [3:0]  pio_pins_oe,
    input  wire        status_out,

    output wire        uart_rx_mux,
    output wire        spi_miso_mux,
    output wire        i2c_scl_in_mux,
    output wire        i2c_sda_in_mux,
    output wire [3:0]  pio_pins_in_mux,
    output wire        timer_cap_mux,
    output wire        ppi_gpio_mux
);

    // Output function IDs (4-bit PSEL)
    localparam [3:0] FN_GPIO   = 4'd0;
    localparam [3:0] FN_UART_TX= 4'd1;
    localparam [3:0] FN_SPI_SCK= 4'd2;
    localparam [3:0] FN_SPI_MOS= 4'd3;
    localparam [3:0] FN_SPI_CS = 4'd4;
    localparam [3:0] FN_I2C_SCL= 4'd5;
    localparam [3:0] FN_I2C_SDA= 4'd6;
    /* verilator lint_off UNUSEDPARAM */
    localparam [3:0] FN_PWM0   = 4'd7;
    localparam [3:0] FN_PWM1   = 4'd8;
    localparam [3:0] FN_PIO0   = 4'd9;
    localparam [3:0] FN_PIO1   = 4'd10;
    localparam [3:0] FN_PIO2   = 4'd11;
    localparam [3:0] FN_PIO3   = 4'd12;
    localparam [3:0] FN_STATUS = 4'd13;
    localparam [3:0] FN_ZERO   = 4'd14;
    localparam [3:0] FN_HIZ    = 4'd15;
    /* verilator lint_on UNUSEDPARAM */

    reg [3:0] out_sel0, out_sel1, out_sel2, out_sel3, out_sel4, out_sel5, out_sel6;
    reg [2:0] in_sel0, in_sel1, in_sel2, in_sel3, in_sel4, in_sel5, in_sel6, in_sel7;
    reg [6:0] gpio_out;
    reg [1:0] gpio_oe;

    wire [7:0] in_sources = {
        1'b1,
        1'b0,
        pio_pins_out[0],
        pin_uio_6_in,
        pin_uio_5_in,
        pin_ui_7,
        pin_ui_3,
        pin_ui_1
    };

    assign uart_rx_mux     = in_sources[in_sel0];
    assign spi_miso_mux    = in_sources[in_sel1];
    assign i2c_scl_in_mux  = in_sources[in_sel2];
    assign i2c_sda_in_mux  = in_sources[in_sel3];
    assign pio_pins_in_mux = {pin_uio_6_in, pin_uio_5_in, in_sources[in_sel5], in_sources[in_sel4]};
    assign timer_cap_mux   = in_sources[in_sel6];
    assign ppi_gpio_mux    = in_sources[in_sel7];

    wire [15:0] fn_out = {
        1'b0, 1'b0, status_out, pio_pins_out,
        pwm1_out, pwm0_out, 1'b0, 1'b0,
        spi_cs_n, spi_mosi, spi_sck, uart_tx, 1'b0
    };
    wire [15:0] fn_oe = {
        1'b0, 1'b1, 1'b1, pio_pins_oe,
        1'b1, 1'b1, i2c_sda_drive_low, i2c_scl_drive_low,
        1'b1, 1'b1, 1'b1, 1'b1, 1'b0
    };
    wire _unused_fn0 = &{1'b0, fn_out[0], fn_oe[0]};

    wire [1:0] eo0 = (out_sel0 == FN_GPIO) ? {1'b1, gpio_out[0]}        : {fn_oe[out_sel0], fn_out[out_sel0]};
    wire [1:0] eo1 = (out_sel1 == FN_GPIO) ? {1'b1, gpio_out[1]}        : {fn_oe[out_sel1], fn_out[out_sel1]};
    wire [1:0] eo2 = (out_sel2 == FN_GPIO) ? {1'b1, gpio_out[2]}        : {fn_oe[out_sel2], fn_out[out_sel2]};
    wire [1:0] eo3 = (out_sel3 == FN_GPIO) ? {1'b1, gpio_out[3]}        : {fn_oe[out_sel3], fn_out[out_sel3]};
    wire [1:0] eo4 = (out_sel4 == FN_GPIO) ? {1'b1, gpio_out[4]}        : {fn_oe[out_sel4], fn_out[out_sel4]};
    wire [1:0] eo5 = (out_sel5 == FN_GPIO) ? {gpio_oe[0], gpio_out[5]}  : {fn_oe[out_sel5], fn_out[out_sel5]};
    wire [1:0] eo6 = (out_sel6 == FN_GPIO) ? {gpio_oe[1], gpio_out[6]}  : {fn_oe[out_sel6], fn_out[out_sel6]};
    wire _unused_oe = &{1'b0, eo0[1], eo1[1], eo2[1], eo3[1], eo4[1]};

    always @(*) begin
        pin_uo_3      = eo0[0];
        pin_uo_4      = eo1[0];
        pin_uo_5      = eo2[0];
        pin_uo_6      = eo3[0];
        pin_uo_7      = eo4[0];
        pin_uio_5_out = eo5[0];
        pin_uio_5_oe  = eo5[1];
        pin_uio_6_out = eo6[0];
        pin_uio_6_oe  = eo6[1];
    end

    always @(*) begin
        case (req_op)
            4'd0, 4'd1, 4'd2, 4'd3,
            4'd4, 4'd5, 4'd6: resp_data16 = {12'd0, req_data[3:0]};
            4'd7:             resp_data16 = {8'd0, req_data};
            4'd8:             resp_data16 = {9'd0, req_data[6:0]};
            4'd9:             resp_data16 = {14'd0, req_data[1:0]};
            4'd10:            resp_data16 = {11'd0, pin_uio_6_in, pin_uio_5_in, pin_ui_7, pin_ui_3, pin_ui_1};
            4'd11: begin
                case (req_data[2:0])
                    3'd0:    resp_data16 = {12'd0, out_sel0};
                    3'd1:    resp_data16 = {12'd0, out_sel1};
                    3'd2:    resp_data16 = {12'd0, out_sel2};
                    3'd3:    resp_data16 = {12'd0, out_sel3};
                    3'd4:    resp_data16 = {12'd0, out_sel4};
                    3'd5:    resp_data16 = {12'd0, out_sel5};
                    3'd6:    resp_data16 = {12'd0, out_sel6};
                    default: resp_data16 = 16'd0;
                endcase
            end
            default:          resp_data16 = 16'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_sel0  <= FN_SPI_SCK; // uo[3]
            out_sel1  <= FN_UART_TX; // uo[4]
            out_sel2  <= FN_SPI_MOS; // uo[5]
            out_sel3  <= FN_SPI_CS;  // uo[6]
            out_sel4  <= FN_STATUS;  // uo[7]
            out_sel5  <= FN_I2C_SCL; // uio[5]
            out_sel6  <= FN_I2C_SDA; // uio[6]

            in_sel0   <= 3'd1; // uart_rx  <- ui[3]
            in_sel1   <= 3'd0; // spi_miso <- ui[1]
            in_sel2   <= 3'd3; // i2c_scl  <- uio[5]
            in_sel3   <= 3'd4; // i2c_sda  <- uio[6]
            in_sel4   <= 3'd2; // pio_in0  <- ui[7]
            in_sel5   <= 3'd0; // pio_in1  <- ui[1]
            in_sel6   <= 3'd2; // tcap     <- ui[7]
            in_sel7   <= 3'd2; // ppi_gpio <- ui[7]

            gpio_out  <= 7'd0;
            gpio_oe   <= 2'd0;
        end else if (req_valid) begin
            case (req_op)
                4'd0: out_sel0 <= req_data[3:0];
                4'd1: out_sel1 <= req_data[3:0];
                4'd2: out_sel2 <= req_data[3:0];
                4'd3: out_sel3 <= req_data[3:0];
                4'd4: out_sel4 <= req_data[3:0];
                4'd5: out_sel5 <= req_data[3:0];
                4'd6: out_sel6 <= req_data[3:0];
                4'd7: begin
                    case (req_data[6:4])
                        3'd0: in_sel0 <= req_data[2:0];
                        3'd1: in_sel1 <= req_data[2:0];
                        3'd2: in_sel2 <= req_data[2:0];
                        3'd3: in_sel3 <= req_data[2:0];
                        3'd4: in_sel4 <= req_data[2:0];
                        3'd5: in_sel5 <= req_data[2:0];
                        3'd6: in_sel6 <= req_data[2:0];
                        3'd7: in_sel7 <= req_data[2:0];
                    endcase
                end
                4'd8: gpio_out <= req_data[6:0];
                4'd9: gpio_oe  <= req_data[1:0];
                default: begin end
            endcase
        end
    end

endmodule
