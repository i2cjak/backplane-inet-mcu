`default_nettype none

module inet_alu_formal (
    input wire [3:0]  op,
    input wire [15:0] a,
    input wire [15:0] b
);

    wire [15:0] res;
    wire [15:0] res_swapped;

    inet_alu dut (
        .op (op),
        .a  (a),
        .b  (b),
        .res(res)
    );

    inet_alu dut_swapped (
        .op (op),
        .a  (b),
        .b  (a),
        .res(res_swapped)
    );

    // Independent bit-exact reference expressions matching spec/isa.py::eval_alu
    wire [3:0]  sh = b[3:0];
    wire [31:0] sext_a = {{16{a[15]}}, a};
    wire [31:0] sar_ref = sext_a >> sh;
    wire [15:0] flip_a = {~a[15], a[14:0]};
    wire [15:0] flip_b = {~b[15], b[14:0]};

    always @(*) begin
        case (op)
            4'h0: assert (res == (a + b) && res == res_swapped);             // ADD
            4'h1: assert (res == (a - b));                                   // SUB
            4'h2: assert (res == (a & b) && res == res_swapped);             // AND
            4'h3: assert (res == (a | b) && res == res_swapped);             // OR
            4'h4: assert (res == (a ^ b) && res == res_swapped);             // XOR
            4'h5: assert (res == ((a << sh) & 16'hFFFF));                    // SHL
            4'h6: assert (res == (a >> sh));                                 // SHR
            4'h7: assert (res == sar_ref[15:0] && res[15] == a[15]);         // SAR
            4'h8: assert (res == {15'd0, (a == b)} && res == res_swapped);   // EQ
            4'h9: assert (res == {15'd0, (a != b)} && res == res_swapped);   // NE
            4'hA: assert (res == {15'd0, (a < b)});                          // LTU
            4'hB: assert (res == {15'd0, (flip_a < flip_b)});                // LTS
            4'hC: assert (res == {15'd0, (a <= b)});                         // LEU
            4'hD: assert (res == ((a * b) & 16'hFFFF) && res == res_swapped);// MUL
            4'hE: assert (res == b);                                         // SEQ
            4'hF: assert (res == {a[7:0], b[7:0]});                          // PACK
        endcase
    end

endmodule
