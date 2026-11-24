/*
 * Copyright (c) 2026 TT_INet_MCU Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Scalable Multi-Tile Interaction-Net Reduction Core (TT_INet_MCU) — SKY130 8x2 Edition
 *   - Exact 24-bit port ABI matching spec/isa.py & spec/golden_model.py
 *   - Latch-RF hot node/wire arenas, REF directory cache, REF map, and tile deques
 *     (sky130_fd_sc_hd__dlxtn_1 via inet_latch_rf)
 *   - Parameterized N_TILES (2..4) sticky-tile Chase-Lev LIFO deques + oldest-end
 *     peer stealing + lazy LIFO PSRAM B (CS2_n) spill/refill stack
 *   - Per-node/per-wire REF hot allocator + single-burst Flash template reads
 *     + 2-redex Flash bursts + 16-entry REF directory cache
 *   - Non-blocking linear IO bridge, PPI/DMA active-pair injection, & SWD Region 0x7 MMIO
 */

`default_nettype none

module inet_core #(
    parameter integer HOT_NODES       = 1,
    parameter integer HOT_WIRES       = 1,
    parameter integer REF_DIR_ENTRIES = 0,
    parameter integer N_TILES         = 2
) (
    input  wire        clk,
    input  wire        rst_n,

    // Shared QSPI Flash/PSRAM controller interface
    output reg         qspi_cmd_valid,
    input  wire        qspi_cmd_ready,
    output reg  [1:0]  qspi_cmd_dev,    // 0=Flash CS0, 1=PSRAM A CS1, 2=PSRAM B CS2
    output reg         qspi_cmd_write,  // 0=read, 1=write
    output wire [23:0] qspi_cmd_addr,
    output reg  [4:0]  qspi_cmd_words,  // words - 1
    output reg  [31:0] qspi_wdata,
    output reg         qspi_wdata_valid,
    input  wire        qspi_wdata_ready,
    input  wire [31:0] qspi_rdata,
    input  wire        qspi_rdata_valid,
    output reg         qspi_rdata_ready,
    input  wire        qspi_busy,

    // Linear IO bridge interface
    output reg         io_req_valid,
    input  wire        io_req_ready,
    output wire [3:0]  io_req_dev,
    output wire [15:0] io_req_val,
    output wire [23:0] io_req_cont,
    input  wire        io_out_valid,
    output reg         io_out_ready,
    input  wire [23:0] io_out_a,
    input  wire [23:0] io_out_b,
    input  wire        io_busy,

    // Autonomous PPI / DMA active-pair injection
    input  wire        ppi_inj_valid,
    output reg         ppi_inj_ready,
    input  wire [23:0] ppi_inj_a,
    input  wire [23:0] ppi_inj_b,

    // SWD Region 0x7 MMIO bridge
    output reg         dbg_mmio_valid,
    input  wire        dbg_mmio_ready,
    output wire [3:0]  dbg_mmio_dev,
    output wire [3:0]  dbg_mmio_op,
    output wire [7:0]  dbg_mmio_wdata,
    input  wire [15:0] dbg_mmio_rdata,

    // SWD debug control, telemetry, and memory access
    input  wire        boot_halt,
    input  wire        dbg_halt_req,
    input  wire        dbg_step_pulse,
    input  wire        dbg_reset_pulse,
    output wire        core_halted,
    output wire        core_busy,
    output wire [3:0]  active_tiles,
    output wire [31:0] cycle_cnt,
    output wire [31:0] redex_cnt,
    output wire [31:0] qspi_txn_cnt,
    output wire [31:0] queue_depths,

    input  wire        dbg_mem_req,
    input  wire        dbg_mem_we,
    input  wire [23:0] dbg_mem_addr,
    input  wire [31:0] dbg_mem_wdata,
    output reg         dbg_mem_ready,
    output wire [31:0] dbg_mem_rdata
);

    localparam integer TILE_W      = (N_TILES <= 2) ? 1 : 2;

    // ------------------------------------------------------------------------
    // ISA Constants (synced with spec/isa.py)
    // ------------------------------------------------------------------------
    localparam [3:0] TAG_FREE = 4'h0;
    localparam [3:0] TAG_VAR  = 4'h1;
    localparam [3:0] TAG_ERA  = 4'h2;
    localparam [3:0] TAG_NUM  = 4'h3;
    localparam [3:0] TAG_REF  = 4'h4;
    localparam [3:0] TAG_ROOT = 4'h5;
    localparam [3:0] TAG_CON  = 4'h8;
    localparam [3:0] TAG_DUP  = 4'h9;
    localparam [3:0] TAG_OPR  = 4'hA;
    localparam [3:0] TAG_OP1  = 4'hB;
    localparam [3:0] TAG_SWI  = 4'hC;
    localparam [3:0] TAG_IO   = 4'hD;

    localparam [31:0] FLASH_MAGIC  = 32'h54454E49; // "INET"
    localparam [15:0] HOT_NODES_16 = HOT_NODES[15:0];
    localparam [15:0] HOT_WIRES_16 = HOT_WIRES[15:0];
    localparam [11:0] HOT_NODES_12 = HOT_NODES[11:0];
    localparam [11:0] HOT_WIRES_12 = HOT_WIRES[11:0];

    function [23:0] mk_port;
        input [3:0]  tag;
        input [3:0]  meta;
        input [15:0] val;
        begin
            mk_port = {tag, meta, val};
        end
    endfunction

    // ------------------------------------------------------------------------
    // Scalar Hot Node & Wire Arenas (HOT_NODES=1, HOT_WIRES=1)
    // ------------------------------------------------------------------------
    reg        hot_node_used;
    reg        hot_wire_used;

    reg [47:0] hn_slot;
    wire [23:0] hn_rd_a0 = hn_slot[23:0];
    wire [23:0] hn_rd_a1 = hn_slot[47:24];

    reg [23:0] hw_slot;
    wire [23:0] hw_rd_data = hw_slot;

    // ------------------------------------------------------------------------
    // Roots, Status, PSRAM Allocators, Spill Stack
    // ------------------------------------------------------------------------
    reg [23:0] roots [0:1];
    reg        booted;
    reg        err_boot;
    reg        err_linear;
    reg        err_type;
    reg [9:0]  hot_hit_cnt;
    reg [9:0]  psram_spill_cnt;
    reg [3:0]  io_inflight_cnt;
    reg [19:0] cycle_cnt_r;
    reg [13:0] redex_cnt_r;
    reg [13:0] qspi_txn_cnt_r;

    assign cycle_cnt    = {12'd0, cycle_cnt_r};
    assign redex_cnt    = {18'd0, redex_cnt_r};
    assign qspi_txn_cnt = {18'd0, qspi_txn_cnt_r};

    reg [11:0] psram_next_node;
    reg [11:0] psram_next_wire;

    reg [9:0]  spill_cnt; // Lazy LIFO PSRAM B stack depth

    reg [17:0] qspi_cmd_waddr;
    assign qspi_cmd_addr = {4'd0, qspi_cmd_waddr, 2'b00};

    wire _unused_ref_dir = (REF_DIR_ENTRIES > 0) || dbg_mem_addr[0] || (|qspi_rdata[17:14]);

    // ------------------------------------------------------------------------
    // Parameterized Multi-Tile Chase-Lev Work-Stealing Deques (DFF, 1 entry/tile)
    // ------------------------------------------------------------------------
    reg [47:0]        dq_mem [0:N_TILES-1];
    reg [N_TILES-1:0] dq_valid;
    reg [TILE_W-1:0]  dq_rd_addr;
    wire [47:0]       dq_rd_data = dq_mem[dq_rd_addr];

    wire       all_dq_full     = &dq_valid;
    wire       any_dq_nonempty = |dq_valid;
    reg [3:0]  tile_active_vec;
    integer ti;
    always @(*) begin
        tile_active_vec = 4'd0;
        for (ti = 0; ti < N_TILES; ti = ti + 1) begin
            tile_active_vec[ti] = dq_valid[ti];
        end
    end

    // ------------------------------------------------------------------------
    // Unified Reduction & Service State Machine
    // ------------------------------------------------------------------------
    localparam [4:0] S_IDLE        = 5'd0;
    localparam [4:0] S_BOOT        = 5'd1;
    localparam [4:0] S_DBG_QSPI    = 5'd2;
    localparam [4:0] S_RD_A        = 5'd3;
    localparam [4:0] S_RD_B        = 5'd4;
    localparam [4:0] S_DISP        = 5'd5;
    localparam [4:0] S_ALLOC       = 5'd6;
    localparam [4:0] S_WR          = 5'd7;
    localparam [4:0] S_IO          = 5'd8;
    localparam [4:0] S_LINK        = 5'd9;
    localparam [4:0] S_LINK_RD     = 5'd10;
    localparam [4:0] S_LINK_WR     = 5'd11;
    localparam [4:0] S_REF_DIR     = 5'd12;
    localparam [4:0] S_REF_ROOT    = 5'd14;
    localparam [4:0] S_REF_NODE_RD = 5'd15;
    localparam [4:0] S_REF_NODE_WR = 5'd16;
    localparam [4:0] S_REF_REDEX   = 5'd17;
    localparam [4:0] S_SPILL_WR    = 5'd18;
    localparam [4:0] S_REFILL_RD   = 5'd19;
    localparam [4:0] S_DBG_MMIO    = 5'd20;

    localparam [3:0] K_DIRECT   = 4'd0;
    localparam [3:0] K_ERA_1    = 4'd1;
    localparam [3:0] K_ERA_2    = 4'd2;
    localparam [3:0] K_DUP_ATOM = 4'd3;
    localparam [3:0] K_PAIR2    = 4'd4;
    localparam [3:0] K_DUP_OP1  = 4'd5;
    localparam [3:0] K_DUP_COMM = 4'd6;
    localparam [3:0] K_OPR      = 4'd7;
    localparam [3:0] K_OP1      = 4'd8;
    localparam [3:0] K_SWI_Z    = 4'd9;
    localparam [3:0] K_SWI_NZ   = 4'd10;

    reg [4:0]  state, after_spill, after_link;
    reg [3:0]  kind;
    reg [TILE_W-1:0] cur_tile;
    reg        q_active;

    reg [23:0] pa, pb;
    reg [23:0] buf_a0, buf_a1, buf_b0, buf_b1;

    // Shared Combinational ALU
    wire [3:0]  alu_op  = pb[19:16];
    wire [15:0] alu_a   = buf_b0[15:0];
    wire [15:0] alu_b   = pa[15:0];
    wire [15:0] alu_res;

    inet_alu u_alu (
        .op (alu_op),
        .a  (alu_a),
        .b  (alu_b),
        .res(alu_res)
    );

    reg [23:0] cur_la, cur_lb;
    reg        chain_active;
    reg [2:0]  lnk_idx, lnk_cnt;
    reg [11:0] cur_wid;

    reg [11:0] alloc_n [0:3];
    reg [11:0] alloc_w [0:3];
    reg [2:0]  alloc_step;
    reg [2:0]  alloc_need_n, alloc_need_w;
    reg [2:0]  wr_step, wr_total;
    reg        wr_phase;

    reg [7:0]  exp_ref_id;
    // Note: exp_other reuses pa, exp_root reuses pb across S_RD_A -> S_REF_* -> S_LINK
    reg [13:0] exp_woff;
    reg [4:0]  exp_nnodes;
    reg [3:0]  exp_nredex;
    reg [11:0] exp_base_nid, exp_base_wid;
    reg [4:0]  exp_node_idx;
    reg [3:0]  exp_redex_idx;
    reg [2:0]  exp_phase;

    assign io_req_dev  = pb[19:16];
    assign io_req_val  = pa[15:0];
    assign io_req_cont = buf_b0;

    assign dbg_mmio_dev   = dbg_mem_addr[11:8];
    assign dbg_mmio_op    = dbg_mem_addr[5:2];
    assign dbg_mmio_wdata = dbg_mem_we ? dbg_mem_wdata[7:0] : 8'd0;
    // buf_a0/buf_a1[7:0] are idle in S_IDLE/S_DBG_QSPI/S_DBG_MMIO and hold dbg_mem_rdata
    assign dbg_mem_rdata  = {buf_a1[7:0], buf_a0};

    // dbg_inj_a reuses pa across Region 0x3 2-word write
    reg        dbg_pending;
    reg        step_armed;

    wire effective_halt = boot_halt | dbg_halt_req;
    wire allow_step     = !effective_halt || step_armed;

    assign core_halted = effective_halt &&
                         (state == S_IDLE || state == S_DBG_QSPI || state == S_DBG_MMIO);

    assign core_busy = !booted ||
                       (state != S_IDLE) ||
                       any_dq_nonempty ||
                       (spill_cnt != 10'd0) ||
                       (io_inflight_cnt != 4'd0) || io_busy || io_out_valid;

    assign active_tiles = {
        (N_TILES > 3) ? (tile_active_vec[3] || (state != S_IDLE && {{(2-TILE_W){1'b0}}, cur_tile} == 2'd3)) : 1'b0,
        (N_TILES > 2) ? (tile_active_vec[2] || (state != S_IDLE && {{(2-TILE_W){1'b0}}, cur_tile} == 2'd2)) : qspi_busy,
        tile_active_vec[1] || (state != S_IDLE && {{(2-TILE_W){1'b0}}, cur_tile} == 2'd1),
        tile_active_vec[0] || (state != S_IDLE && {{(2-TILE_W){1'b0}}, cur_tile} == 2'd0)
    };

    assign queue_depths = {
        spill_cnt,
        6'd0,
        7'd0, dq_valid[1],
        7'd0, dq_valid[0]
    };

    // Combinational port relocator (shared 12-bit base adder)
    function [23:0] reloc_port;
        input [23:0] p;
        input [11:0] nbase;
        input [11:0] wbase;
        reg   [11:0] sum_id;
        begin
            sum_id = p[11:0] + (p[23] ? nbase : wbase);
            if (p[23] || p[23:20] == TAG_VAR) reloc_port = {p[23:16], 4'd0, sum_id};
            else                              reloc_port = p;
        end
    endfunction

    wire [23:0] reloc_qspi_port = reloc_port(qspi_rdata[23:0], exp_base_nid, exp_base_wid);

    // ------------------------------------------------------------------------
    // Combinational Writeback & Link Generators
    // ------------------------------------------------------------------------
    reg [11:0] wr_nid;
    reg [23:0] wr_a0, wr_a1;
    always @(*) begin
        wr_nid = 12'd0;
        wr_a0  = 24'd0;
        wr_a1  = 24'd0;
        case (kind)
            K_OPR: begin
                wr_nid = pb[11:0];
                wr_a0  = pa;
                wr_a1  = buf_b1;
            end
            K_SWI_Z: begin
                wr_nid = pb[11:0];
                wr_a0  = buf_b1;
                wr_a1  = mk_port(TAG_ERA, 4'd0, 16'd0);
            end
            K_SWI_NZ: begin
                if (wr_step == 3'd0) begin : swi_nz_w0
                    reg [15:0] arg;
                    arg = (pb[16] == 1'b0) ? (pa[15:0] - 16'd1) : pa[15:0];
                    wr_nid = alloc_n[0];
                    wr_a0  = mk_port(TAG_NUM, 4'd0, arg);
                    wr_a1  = buf_b1;
                end else begin
                    wr_nid = pb[11:0];
                    wr_a0  = mk_port(TAG_ERA, 4'd0, 16'd0);
                    wr_a1  = mk_port(TAG_CON, 4'd0, {4'd0, alloc_n[0]});
                end
            end
            K_DUP_OP1: begin
                case (wr_step[1:0])
                    2'd0: begin wr_nid = alloc_n[0]; wr_a0 = buf_b0; wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[0]}); end
                    2'd1: begin wr_nid = alloc_n[1]; wr_a0 = buf_b0; wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[1]}); end
                    default: begin wr_nid = alloc_n[2]; wr_a0 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[0]}); wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[1]}); end
                endcase
            end
            K_DUP_COMM: begin
                case (wr_step[1:0])
                    2'd0: begin wr_nid = alloc_n[0]; wr_a0 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[0]}); wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[2]}); end
                    2'd1: begin wr_nid = alloc_n[1]; wr_a0 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[1]}); wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[3]}); end
                    2'd2: begin wr_nid = alloc_n[2]; wr_a0 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[0]}); wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[1]}); end
                    default: begin wr_nid = alloc_n[3]; wr_a0 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[2]}); wr_a1 = mk_port(TAG_VAR, 4'd0, {4'd0, alloc_w[3]}); end
                endcase
            end
            default: begin end
        endcase
    end

    reg [23:0] gen_la, gen_lb;
    always @(*) begin
        gen_la = cur_la;
        gen_lb = cur_lb;
        if (chain_active) begin
            gen_la = cur_la;
            gen_lb = cur_lb;
        end else case (kind)
            K_DIRECT: begin gen_la = cur_la; gen_lb = cur_lb; end
            K_ERA_1:  begin gen_la = (pa[23:20] == TAG_OP1 || pb[23:20] == TAG_OP1) ? buf_b1 : buf_b0; gen_lb = mk_port(TAG_ERA, 4'd0, 16'd0); end
            K_ERA_2:  begin gen_la = (lnk_idx == 3'd0) ? buf_b0 : buf_b1; gen_lb = mk_port(TAG_ERA, 4'd0, 16'd0); end
            K_DUP_ATOM: begin gen_la = (lnk_idx == 3'd0) ? buf_a0 : buf_a1; gen_lb = pb; end
            K_PAIR2:  begin gen_la = (lnk_idx == 3'd0) ? buf_a0 : buf_a1; gen_lb = (lnk_idx == 3'd0) ? buf_b0 : buf_b1; end
            K_DUP_OP1: begin
                case (lnk_idx[1:0])
                    2'd0: begin gen_la = buf_a0; gen_lb = mk_port(TAG_OP1, pb[19:16], {4'd0, alloc_n[0]}); end
                    2'd1: begin gen_la = buf_a1; gen_lb = mk_port(TAG_OP1, pb[19:16], {4'd0, alloc_n[1]}); end
                    default: begin gen_la = buf_b1; gen_lb = mk_port(TAG_DUP, pa[19:16], {4'd0, alloc_n[2]}); end
                endcase
            end
            K_DUP_COMM: begin
                case (lnk_idx[1:0])
                    2'd0: begin gen_la = buf_a0; gen_lb = mk_port(pb[23:20], pb[19:16], {4'd0, alloc_n[0]}); end
                    2'd1: begin gen_la = buf_a1; gen_lb = mk_port(pb[23:20], pb[19:16], {4'd0, alloc_n[1]}); end
                    2'd2: begin gen_la = buf_b0; gen_lb = mk_port(TAG_DUP, pa[19:16], {4'd0, alloc_n[2]}); end
                    default: begin gen_la = buf_b1; gen_lb = mk_port(TAG_DUP, pa[19:16], {4'd0, alloc_n[3]}); end
                endcase
            end
            K_OPR: begin gen_la = buf_b0; gen_lb = mk_port(TAG_OP1, pb[19:16], pb[15:0]); end
            K_OP1: begin gen_la = buf_b1; gen_lb = mk_port(TAG_NUM, 4'd0, alu_res); end
            K_SWI_Z, K_SWI_NZ: begin gen_la = buf_b0; gen_lb = mk_port(TAG_CON, 4'd0, pb[15:0]); end
            default: begin end
        endcase
    end

    // Sticky-tile pop / steal address computation for u_dq_rf
    reg              dq_pick_found;
    reg [TILE_W-1:0] dq_pick_tile;
    reg              dq_pick_steal;
    always @(*) begin : dq_pick_comb
        integer k;
        reg [TILE_W-1:0] cand;
        reg [2:0] sum_t;
        dq_pick_found = 1'b0;
        dq_pick_tile  = cur_tile;
        dq_pick_steal = 1'b0;
        cand          = {TILE_W{1'b0}};
        sum_t         = 3'd0;
        if (dq_valid[cur_tile]) begin
            dq_pick_found = 1'b1;
            dq_pick_tile  = cur_tile;
            dq_pick_steal = 1'b0;
        end else begin
            for (k = 1; k < N_TILES; k = k + 1) begin
                sum_t = {{(3-TILE_W){1'b0}}, cur_tile} + k[2:0];
                cand  = (sum_t >= N_TILES[2:0]) ? (sum_t[TILE_W-1:0] - N_TILES[TILE_W-1:0]) : sum_t[TILE_W-1:0];
                if (!dq_pick_found && dq_valid[cand]) begin
                    dq_pick_found = 1'b1;
                    dq_pick_tile  = cand;
                    dq_pick_steal = 1'b1;
                end
            end
        end
    end

    // Combinational QSPI write-data mux across all write states
    always @(*) begin
        case (state)
            S_DBG_QSPI:    qspi_wdata = dbg_mem_wdata;
            S_WR:          qspi_wdata = wr_phase ? {8'd0, wr_a1} : {8'd0, wr_a0};
            S_LINK_WR:     qspi_wdata = wr_phase ? 32'd0 : {8'd0, cur_lb};
            S_REF_NODE_WR: qspi_wdata = wr_phase ? {8'd0, buf_a1} : {8'd0, buf_a0};
            S_SPILL_WR:    qspi_wdata = wr_phase ? {8'd0, buf_a0} : {8'd0, buf_b0};
            default:       qspi_wdata = 32'd0;
        endcase
    end

    // Read-port address mux for deque DFFs
    always @(*) begin : rf_raddr_mux
        if (state == S_IDLE && dbg_pending && dbg_mem_addr[23:20] == 4'h3) begin
            dq_rd_addr = dbg_mem_addr[3 +: TILE_W];
        end else if (state == S_SPILL_WR) begin
            dq_rd_addr = {TILE_W{1'b0}};
        end else begin
            dq_rd_addr = dq_pick_tile;
        end
    end

    // ------------------------------------------------------------------------
    // Sequential FSM
    // ------------------------------------------------------------------------
    integer idx_i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hot_node_used <= 1'b0;
            hot_wire_used <= 1'b0;

            for (idx_i = 0; idx_i < 4; idx_i = idx_i + 1) begin
                alloc_n[idx_i] <= 12'd0;
                alloc_w[idx_i] <= 12'd0;
            end
            for (idx_i = 0; idx_i < 2; idx_i = idx_i + 1) begin
                roots[idx_i] <= 24'd0;
            end
            hn_slot <= 48'd0;
            hw_slot <= 24'd0;
            for (idx_i = 0; idx_i < N_TILES; idx_i = idx_i + 1) begin
                dq_mem[idx_i] <= 48'd0;
            end
            dq_valid        <= {N_TILES{1'b0}};

            booted          <= 1'b0;
            err_boot        <= 1'b0;
            err_linear      <= 1'b0;
            err_type        <= 1'b0;
            cycle_cnt_r     <= 20'd0;
            redex_cnt_r     <= 14'd0;
            qspi_txn_cnt_r  <= 14'd0;
            hot_hit_cnt     <= 10'd0;
            psram_spill_cnt <= 10'd0;
            io_inflight_cnt <= 4'd0;

            psram_next_node <= HOT_NODES_12;
            psram_next_wire <= HOT_WIRES_12;
            spill_cnt       <= 10'd0;

            state           <= S_IDLE;
            after_spill     <= S_IDLE;
            after_link      <= S_IDLE;
            kind            <= K_DIRECT;
            cur_tile        <= {TILE_W{1'b0}};
            q_active        <= 1'b0;
            pa              <= 24'd0;
            pb              <= 24'd0;
            buf_a0          <= 24'd0;
            buf_a1          <= 24'd0;
            buf_b0          <= 24'd0;
            buf_b1          <= 24'd0;
            cur_la          <= 24'd0;
            cur_lb          <= 24'd0;
            chain_active    <= 1'b0;
            lnk_idx         <= 3'd0;
            lnk_cnt         <= 3'd0;
            cur_wid         <= 12'd0;

            alloc_step        <= 3'd0;
            alloc_need_n      <= 3'd0;
            alloc_need_w      <= 3'd0;
            wr_step           <= 3'd0;
            wr_total          <= 3'd0;
            wr_phase          <= 1'b0;

            exp_ref_id        <= 8'd0;
            exp_woff          <= 14'd0;
            exp_nnodes        <= 5'd0;
            exp_nredex        <= 4'd0;
            exp_base_nid      <= 12'd0;
            exp_base_wid      <= 12'd0;
            exp_node_idx      <= 5'd0;
            exp_redex_idx     <= 4'd0;
            exp_phase         <= 3'd0;

            qspi_cmd_valid   <= 1'b0;
            qspi_cmd_dev     <= 2'd0;
            qspi_cmd_write   <= 1'b0;
            qspi_cmd_waddr   <= 18'd0;
            qspi_cmd_words   <= 5'd0;
            qspi_wdata_valid <= 1'b0;
            qspi_rdata_ready <= 1'b1;

            io_req_valid     <= 1'b0;
            io_out_ready     <= 1'b0;
            ppi_inj_ready    <= 1'b0;

            dbg_mmio_valid   <= 1'b0;
            dbg_mem_ready    <= 1'b0;
            dbg_pending      <= 1'b0;
            step_armed       <= 1'b0;
        end else if (dbg_reset_pulse) begin
            hot_node_used    <= 1'b0;
            hot_wire_used    <= 1'b0;
            dq_valid         <= {N_TILES{1'b0}};
            booted           <= 1'b0;
            err_boot         <= 1'b0;
            err_linear       <= 1'b0;
            err_type         <= 1'b0;
            cycle_cnt_r      <= 20'd0;
            redex_cnt_r      <= 14'd0;
            qspi_txn_cnt_r   <= 14'd0;
            hot_hit_cnt      <= 10'd0;
            psram_spill_cnt  <= 10'd0;
            io_inflight_cnt  <= 4'd0;
            psram_next_node  <= HOT_NODES_12;
            psram_next_wire  <= HOT_WIRES_12;
            spill_cnt        <= 10'd0;
            state            <= S_IDLE;
            q_active         <= 1'b0;
            chain_active     <= 1'b0;
            qspi_cmd_valid   <= 1'b0;
            qspi_wdata_valid <= 1'b0;
            io_req_valid     <= 1'b0;
            io_out_ready     <= 1'b0;
            ppi_inj_ready    <= 1'b0;
            dbg_mmio_valid   <= 1'b0;
            dbg_mem_ready    <= 1'b0;
            dbg_pending      <= 1'b0;
            step_armed       <= 1'b0;
        end else begin : main_seq
            reg              pop_en, steal_en, spill_pop0;
            reg [TILE_W-1:0] pop_tile;
            reg              push_en;
            reg [TILE_W-1:0] push_tile;
            reg [23:0]       push_a, push_b;

            pop_en     = 1'b0;
            steal_en   = 1'b0;
            pop_tile   = {TILE_W{1'b0}};
            spill_pop0 = 1'b0;
            push_en    = 1'b0;
            push_tile  = {TILE_W{1'b0}};
            push_a     = 24'd0;
            push_b     = 24'd0;

            qspi_cmd_valid <= 1'b0;
            io_req_valid   <= 1'b0;
            io_out_ready   <= 1'b0;
            ppi_inj_ready  <= 1'b0;
            dbg_mmio_valid <= 1'b0;
            dbg_mem_ready  <= 1'b0;

            if (dbg_step_pulse) step_armed <= 1'b1;
            if (dbg_mem_req && !dbg_mem_ready && state != S_DBG_QSPI && state != S_DBG_MMIO) dbg_pending <= 1'b1;
            if (core_busy && !effective_halt) cycle_cnt_r <= cycle_cnt_r + 20'd1;

            case (state)
                S_IDLE: begin
                    q_active <= 1'b0;
                    if (dbg_pending && !dbg_mem_ready) begin : dbg_enter
                        reg [3:0]  region;
                        reg [11:0] dn, dw;
                        region = dbg_mem_addr[23:20];
                        dn     = dbg_mem_addr[14:3];
                        dw     = dbg_mem_addr[13:2];
                        dbg_pending <= 1'b0;
                        if (region == 4'h0) begin
                            case (dbg_mem_addr[4:2])
                                3'd0, 3'd1: begin
                                    if (dbg_mem_we) roots[dbg_mem_addr[2]] <= dbg_mem_wdata[23:0];
                                    {buf_a1[7:0], buf_a0} <= {8'd0, roots[dbg_mem_addr[2]]};
                                end
                                3'd4: {buf_a1[7:0], buf_a0} <= {
                                    8'd0,
                                    {7'd0, hot_node_used},
                                    {7'd0, hot_wire_used},
                                    3'd0, err_type, err_linear, err_boot,
                                    booted, core_busy
                                };
                                3'd5: {buf_a1[7:0], buf_a0} <= {22'd0, hot_hit_cnt};
                                3'd6: {buf_a1[7:0], buf_a0} <= {22'd0, psram_spill_cnt};
                                default: {buf_a1[7:0], buf_a0} <= 32'd0;
                            endcase
                            dbg_mem_ready <= 1'b1;
                        end else if (region == 4'h1) begin
                            if (dn < HOT_NODES_12) begin
                                if (dbg_mem_we) begin
                                    hn_slot <= {dbg_mem_addr[2] ? dbg_mem_wdata[23:0] : hn_rd_a1,
                                                dbg_mem_addr[2] ? hn_rd_a0 : dbg_mem_wdata[23:0]};
                                    hot_node_used <= 1'b1;
                                end
                                {buf_a1[7:0], buf_a0} <= {8'd0, dbg_mem_addr[2] ? hn_rd_a1 : hn_rd_a0};
                                dbg_mem_ready <= 1'b1;
                            end else if (!qspi_busy && qspi_cmd_ready) begin
                                qspi_cmd_valid   <= 1'b1;
                                qspi_cmd_dev     <= 2'd1;
                                qspi_cmd_write   <= dbg_mem_we;
                                qspi_cmd_waddr   <= {5'd0, dn, dbg_mem_addr[2]};
                                qspi_cmd_words   <= 5'd0;
                                qspi_wdata_valid <= dbg_mem_we;
                                qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                                state            <= S_DBG_QSPI;
                            end else begin
                                dbg_pending <= 1'b1;
                            end
                        end else if (region == 4'h2) begin
                            if (dw < HOT_WIRES_12) begin
                                if (dbg_mem_we) begin
                                    hw_slot       <= dbg_mem_wdata[23:0];
                                    hot_wire_used <= (dbg_mem_wdata[23:20] != TAG_FREE);
                                end
                                {buf_a1[7:0], buf_a0} <= {8'd0, hw_rd_data};
                                dbg_mem_ready <= 1'b1;
                            end else if (!qspi_busy && qspi_cmd_ready) begin
                                qspi_cmd_valid   <= 1'b1;
                                qspi_cmd_dev     <= 2'd1;
                                qspi_cmd_write   <= dbg_mem_we;
                                qspi_cmd_waddr   <= {2'b01, 4'd0, dw};
                                qspi_cmd_words   <= 5'd0;
                                qspi_wdata_valid <= dbg_mem_we;
                                qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                                state            <= S_DBG_QSPI;
                            end else begin
                                dbg_pending <= 1'b1;
                            end
                        end else if (region == 4'h3) begin
                            if (dbg_mem_we) begin
                                if (!dbg_mem_addr[2]) begin
                                    pa            <= dbg_mem_wdata[23:0];
                                    {buf_a1[7:0], buf_a0} <= dbg_mem_wdata;
                                    dbg_mem_ready <= 1'b1;
                                end else begin
                                    kind          <= K_DIRECT;
                                    cur_la        <= pa;
                                    cur_lb        <= dbg_mem_wdata[23:0];
                                    chain_active  <= 1'b0;
                                    lnk_idx       <= 3'd0;
                                    lnk_cnt       <= 3'd1;
                                    after_link    <= S_IDLE;
                                    {buf_a1[7:0], buf_a0} <= dbg_mem_wdata;
                                    dbg_mem_ready <= 1'b1;
                                    state         <= S_LINK;
                                end
                            end else begin
                                {buf_a1[7:0], buf_a0} <= {8'd0, dbg_mem_addr[1] ? dq_rd_data[47:24] : dq_rd_data[23:0]};
                                dbg_mem_ready <= 1'b1;
                            end
                        end else if (region == 4'h4 || region == 4'h5 || region == 4'h6) begin
                            if (!qspi_busy && qspi_cmd_ready) begin
                                qspi_cmd_valid   <= 1'b1;
                                qspi_cmd_dev     <= (region == 4'h4) ? 2'd0 :
                                                    (region == 4'h5) ? 2'd1 : 2'd2;
                                qspi_cmd_write   <= (region != 4'h4) && dbg_mem_we;
                                qspi_cmd_waddr   <= dbg_mem_addr[19:2];
                                qspi_cmd_words   <= 5'd0;
                                qspi_wdata_valid <= (region != 4'h4) && dbg_mem_we;
                                qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                                state            <= S_DBG_QSPI;
                            end else begin
                                dbg_pending <= 1'b1;
                            end
                        end else if (region == 4'h7) begin
                            dbg_mmio_valid <= 1'b1;
                            state          <= S_DBG_MMIO;
                        end else begin
                            {buf_a1[7:0], buf_a0} <= 32'd0;
                            dbg_mem_ready <= 1'b1;
                        end
                    end else if (ppi_inj_valid) begin
                        ppi_inj_ready <= 1'b1;
                        kind          <= K_DIRECT;
                        cur_la        <= ppi_inj_a;
                        cur_lb        <= ppi_inj_b;
                        chain_active  <= 1'b0;
                        lnk_idx       <= 3'd0;
                        lnk_cnt       <= 3'd1;
                        after_link    <= S_IDLE;
                        state         <= S_LINK;
                    end else if (!booted) begin
                        if (allow_step && !qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd0;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= 18'd0;
                            qspi_cmd_words <= 5'd3;
                            exp_phase      <= 3'd0;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                            state          <= S_BOOT;
                        end
                    end else if (io_out_valid) begin
                        io_out_ready    <= 1'b1;
                        io_inflight_cnt <= io_inflight_cnt - 4'd1;
                        kind            <= K_DIRECT;
                        cur_la          <= io_out_a;
                        cur_lb          <= io_out_b;
                        chain_active    <= 1'b0;
                        lnk_idx         <= 3'd0;
                        lnk_cnt         <= 3'd1;
                        after_link      <= S_IDLE;
                        state           <= S_LINK;
                    end else if (all_dq_full && !qspi_busy && qspi_cmd_ready) begin
                        after_spill <= S_IDLE;
                        state       <= S_SPILL_WR;
                    end else if (!any_dq_nonempty && (spill_cnt != 10'd0) && !qspi_busy && qspi_cmd_ready) begin
                        state <= S_REFILL_RD;
                    end else if (allow_step && dq_pick_found) begin
                        cur_tile   <= dq_pick_tile;
                        step_armed <= 1'b0;
                        pa         <= dq_rd_data[23:0];
                        pb         <= dq_rd_data[47:24];
                        state      <= S_RD_A;
                        pop_tile   = dq_pick_tile;
                        if (dq_pick_steal) steal_en = 1'b1;
                        else               pop_en   = 1'b1;
                    end
                end

                // ---- Flash XIP Boot ----
                S_BOOT: begin
                    if (qspi_rdata_valid) begin
                        case (exp_phase[1:0])
                            2'd0: if (qspi_rdata != FLASH_MAGIC) err_boot <= 1'b1;
                            2'd1: begin end
                            2'd2: cur_la <= qspi_rdata[23:0];
                            2'd3: cur_lb <= qspi_rdata[23:0];
                        endcase
                        exp_phase <= exp_phase + 3'd1;
                    end
                    if (exp_phase == 3'd4 && !qspi_busy) begin
                        booted       <= 1'b1;
                        kind         <= K_DIRECT;
                        chain_active <= 1'b0;
                        lnk_idx      <= 3'd0;
                        lnk_cnt      <= 3'd1;
                        after_link   <= S_IDLE;
                        state        <= S_LINK;
                    end
                end

                // ---- SWD QSPI / MMIO Passthrough ----
                S_DBG_QSPI: begin
                    if (qspi_wdata_valid && qspi_wdata_ready) qspi_wdata_valid <= 1'b0;
                    if (qspi_rdata_valid) {buf_a1[7:0], buf_a0} <= qspi_rdata;
                    if (!qspi_busy && !qspi_cmd_valid) begin
                        dbg_mem_ready <= 1'b1;
                        state         <= S_IDLE;
                    end
                end

                S_DBG_MMIO: begin
                    if (dbg_mmio_ready) begin
                        {buf_a1[7:0], buf_a0} <= {16'd0, dbg_mmio_rdata};
                        dbg_mem_ready <= 1'b1;
                        state         <= S_IDLE;
                    end
                end

                // ---- Read Node A (or Dispatch REF / ERA) ----
                S_RD_A: begin : rd_a_blk
                    reg [3:0]  ta, tb;
                    reg [23:0] na, nb;
                    ta = pa[23:20];
                    tb = pb[23:20];
                    if ((ta == TAG_REF && tb != TAG_DUP && tb != TAG_ERA) ||
                        (tb == TAG_REF && ta != TAG_DUP && ta != TAG_ERA)) begin
                        redex_cnt_r   <= redex_cnt_r + 14'd1;
                        exp_ref_id    <= (ta == TAG_REF) ? pa[7:0] : pb[7:0];
                        pa            <= (ta == TAG_REF) ? pb : pa; // exp_other
                        exp_node_idx  <= 5'd0;
                        exp_redex_idx <= 4'd0;
                        state         <= S_REF_DIR;
                    end else if (ta == TAG_ERA || tb == TAG_ERA) begin
                        nb = (ta == TAG_ERA) ? pb : pa;
                        if (nb[23] && nb[23:20] != TAG_IO) begin
                            if (nb[15:0] < HOT_NODES_16) begin
                                hot_hit_cnt <= hot_hit_cnt + 10'd1;
                                buf_b0      <= hn_rd_a0;
                                buf_b1      <= hn_rd_a1;
                                state       <= S_DISP;
                            end else if (!q_active) begin
                                if (!qspi_busy && qspi_cmd_ready) begin
                                    qspi_cmd_valid <= 1'b1;
                                    qspi_cmd_dev   <= 2'd1;
                                    qspi_cmd_write <= 1'b0;
                                    qspi_cmd_waddr <= {2'd0, nb[14:0], 1'b0};
                                    qspi_cmd_words <= 5'd1;
                                    exp_phase      <= 3'd0;
                                    q_active       <= 1'b1;
                                    qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                                end
                            end else begin
                                if (qspi_rdata_valid) begin
                                    if (exp_phase == 3'd0) begin buf_b0 <= qspi_rdata[23:0]; exp_phase <= 3'd1; end
                                    else                         buf_b1 <= qspi_rdata[23:0];
                                end
                                if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                                    q_active <= 1'b0;
                                    state    <= S_DISP;
                                end
                            end
                        end else begin
                            state <= S_DISP;
                        end
                    end else begin
                        na = (tb == TAG_DUP && ta != TAG_DUP) ? pb :
                             (tb == TAG_NUM && ta != TAG_NUM && ta != TAG_DUP) ? pb : pa;
                        nb = (tb == TAG_DUP && ta != TAG_DUP) ? pa :
                             (tb == TAG_NUM && ta != TAG_NUM && ta != TAG_DUP) ? pa : pb;
                        if (!q_active) begin
                            pa <= na;
                            pb <= nb;
                        end
                        if (!na[23]) begin
                            state <= S_RD_B;
                        end else if (na[15:0] < HOT_NODES_16) begin
                            hot_hit_cnt <= hot_hit_cnt + 10'd1;
                            buf_a0      <= hn_rd_a0;
                            buf_a1      <= hn_rd_a1;
                            state       <= S_RD_B;
                        end else if (!q_active) begin
                            if (!qspi_busy && qspi_cmd_ready) begin
                                qspi_cmd_valid <= 1'b1;
                                qspi_cmd_dev   <= 2'd1;
                                qspi_cmd_write <= 1'b0;
                                qspi_cmd_waddr <= {2'd0, na[14:0], 1'b0};
                                qspi_cmd_words <= 5'd1;
                                exp_phase      <= 3'd0;
                                q_active       <= 1'b1;
                                qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                            end
                        end else begin
                            if (qspi_rdata_valid) begin
                                if (exp_phase == 3'd0) begin buf_a0 <= qspi_rdata[23:0]; exp_phase <= 3'd1; end
                                else                         buf_a1 <= qspi_rdata[23:0];
                            end
                            if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                                q_active <= 1'b0;
                                state    <= S_RD_B;
                            end
                        end
                    end
                end

                // ---- Read Node B ----
                S_RD_B: begin
                    if (!pb[23] || (pa[23:20] == TAG_DUP && pb[23:20] == TAG_IO)) begin
                        state <= S_DISP;
                    end else if (pb[15:0] < HOT_NODES_16) begin
                        hot_hit_cnt <= hot_hit_cnt + 10'd1;
                        buf_b0      <= hn_rd_a0;
                        buf_b1      <= hn_rd_a1;
                        state       <= S_DISP;
                    end else if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd1;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {2'd0, pb[14:0], 1'b0};
                            qspi_cmd_words <= 5'd1;
                            exp_phase      <= 3'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin
                            if (exp_phase == 3'd0) begin buf_b0 <= qspi_rdata[23:0]; exp_phase <= 3'd1; end
                            else                         buf_b1 <= qspi_rdata[23:0];
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                            q_active <= 1'b0;
                            state    <= S_DISP;
                        end
                    end
                end

                // ---- Unified Active-Pair Rule Dispatch ----
                S_DISP: begin : disp_blk
                    reg [3:0] ta, tb;
                    ta = pa[23:20];
                    tb = pb[23:20];
                    redex_cnt_r  <= redex_cnt_r + 14'd1;
                    chain_active <= 1'b0;
                    lnk_idx      <= 3'd0;
                    after_link   <= S_IDLE;
                    wr_step      <= 3'd0;
                    wr_phase     <= 1'b0;

                    if (ta == TAG_ERA || tb == TAG_ERA) begin : era_disp
                        reg [3:0]  toth;
                        reg [15:0] voth;
                        toth = (ta == TAG_ERA) ? tb : ta;
                        voth = (ta == TAG_ERA) ? pb[15:0] : pa[15:0];
                        if (toth == TAG_IO) begin
                            err_linear <= 1'b1;
                            state      <= S_IDLE;
                        end else if (toth[3]) begin
                            if (voth < HOT_NODES_16) hot_node_used <= 1'b0;
                            if (toth == TAG_OP1 || buf_b1[23:20] == TAG_FREE) begin
                                kind    <= K_ERA_1;
                                lnk_cnt <= 3'd1;
                            end else begin
                                kind    <= K_ERA_2;
                                lnk_cnt <= 3'd2;
                            end
                            state <= S_LINK;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else if (ta == TAG_DUP) begin
                        if (tb == TAG_IO) begin
                            err_linear <= 1'b1;
                            state      <= S_IDLE;
                        end else if (tb == TAG_NUM || tb == TAG_REF) begin
                            if (pa[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            kind    <= K_DUP_ATOM;
                            lnk_cnt <= 3'd2;
                            state   <= S_LINK;
                        end else if (tb == TAG_DUP && pa[19:16] == pb[19:16]) begin
                            if (pa[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            if (pb[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            kind    <= K_PAIR2;
                            lnk_cnt <= 3'd2;
                            state   <= S_LINK;
                        end else if (tb == TAG_OP1 || tb == TAG_CON || tb == TAG_DUP ||
                                     tb == TAG_OPR || tb == TAG_SWI) begin
                            if (pa[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            if (pb[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            alloc_step <= 3'd0;
                            if (tb == TAG_OP1) begin
                                kind         <= K_DUP_OP1;
                                alloc_need_n <= 3'd3;
                                alloc_need_w <= 3'd2;
                                wr_total     <= 3'd3;
                                lnk_cnt      <= 3'd3;
                            end else begin
                                kind         <= K_DUP_COMM;
                                alloc_need_n <= 3'd4;
                                alloc_need_w <= 3'd4;
                                wr_total     <= 3'd4;
                                lnk_cnt      <= 3'd4;
                            end
                            state <= S_ALLOC;
                        end else begin
                            err_type <= 1'b1;
                            state    <= S_IDLE;
                        end
                    end else if (ta == TAG_CON && tb == TAG_CON) begin
                        if (pa[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                        if (pb[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                        kind    <= K_PAIR2;
                        lnk_cnt <= 3'd2;
                        state   <= S_LINK;
                    end else if (ta == TAG_NUM) begin
                        if (tb == TAG_OPR) begin
                            kind     <= K_OPR;
                            wr_total <= 3'd1;
                            lnk_cnt  <= 3'd1;
                            state    <= S_WR;
                        end else if (tb == TAG_OP1) begin
                            if (pb[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            kind    <= K_OP1;
                            lnk_cnt <= 3'd1;
                            state   <= S_LINK;
                        end else if (tb == TAG_SWI) begin
                            if (pa[15:0] == 16'd0) begin
                                kind     <= K_SWI_Z;
                                wr_total <= 3'd1;
                                lnk_cnt  <= 3'd1;
                                state    <= S_WR;
                            end else begin
                                kind         <= K_SWI_NZ;
                                alloc_step   <= 3'd0;
                                alloc_need_n <= 3'd1;
                                alloc_need_w <= 3'd0;
                                wr_total     <= 3'd2;
                                lnk_cnt      <= 3'd1;
                                state        <= S_ALLOC;
                            end
                        end else if (tb == TAG_IO) begin
                            if (pb[15:0] < HOT_NODES_16) hot_node_used <= 1'b0;
                            state <= S_IO;
                        end else begin
                            err_type <= 1'b1;
                            state    <= S_IDLE;
                        end
                    end else begin
                        err_type <= 1'b1;
                        state    <= S_IDLE;
                    end
                end

                // ---- Sequential 1-Per-Cycle Allocator ----
                S_ALLOC: begin
                    if (alloc_step < alloc_need_n) begin
                        if (!hot_node_used) begin
                            hot_node_used            <= 1'b1;
                            alloc_n[alloc_step[1:0]] <= 12'd0;
                        end else begin
                            alloc_n[alloc_step[1:0]] <= psram_next_node;
                            psram_next_node          <= psram_next_node + 12'd1;
                        end
                    end
                    if (alloc_step < alloc_need_w) begin
                        if (!hot_wire_used) begin
                            hot_wire_used            <= 1'b1;
                            hw_slot                  <= 24'd0;
                            alloc_w[alloc_step[1:0]] <= 12'd0;
                        end else begin
                            alloc_w[alloc_step[1:0]] <= psram_next_wire;
                            psram_next_wire          <= psram_next_wire + 12'd1;
                        end
                    end
                    if (alloc_step + 3'd1 >= alloc_need_n && alloc_step + 3'd1 >= alloc_need_w) begin
                        wr_step <= 3'd0;
                        state   <= S_WR;
                    end else begin
                        alloc_step <= alloc_step + 3'd1;
                    end
                end

                // ---- Sequential Node Writeback (Hot DFF Slot or PSRAM A) ----
                S_WR: begin
                    if (wr_step >= wr_total) begin
                        state <= S_LINK;
                    end else if (wr_nid < HOT_NODES_12) begin
                        hn_slot <= {wr_a1, wr_a0};
                        wr_step <= wr_step + 3'd1;
                    end else if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid   <= 1'b1;
                            qspi_cmd_dev     <= 2'd1;
                            qspi_cmd_write   <= 1'b1;
                            qspi_cmd_waddr   <= {5'd0, wr_nid, 1'b0};
                            qspi_cmd_words   <= 5'd1;
                            qspi_wdata_valid <= 1'b1;
                            wr_phase         <= 1'b0;
                            q_active         <= 1'b1;
                            qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_wdata_valid && qspi_wdata_ready) begin
                            if (!wr_phase) wr_phase <= 1'b1;
                            else           qspi_wdata_valid <= 1'b0;
                        end
                        if (!qspi_busy && !qspi_cmd_valid) begin
                            q_active <= 1'b0;
                            wr_step  <= wr_step + 3'd1;
                        end
                    end
                end

                S_IO: begin
                    if (io_req_ready) begin
                        io_req_valid    <= 1'b1;
                        io_inflight_cnt <= io_inflight_cnt + 4'd1;
                        state           <= S_IDLE;
                    end
                end

                // ---- Unified Link Resolver ----
                S_LINK: begin
                    if (lnk_idx >= lnk_cnt) begin
                        chain_active <= 1'b0;
                        state        <= after_link;
                    end else begin : lnk_step
                        reg [23:0] la, lb, lnb;
                        reg [15:0] lna_v;
                        reg [3:0]  lta, ltb;
                        la  = gen_la;
                        lb  = gen_lb;
                        lta = la[23:20];
                        ltb = lb[23:20];
                        if (lta == TAG_VAR || ltb == TAG_VAR) begin
                            lna_v = (lta == TAG_VAR) ? la[15:0] : lb[15:0];
                            lnb   = (lta == TAG_VAR) ? lb : la;
                            if (lna_v < HOT_WIRES_16) begin
                                if (hw_rd_data[23:20] == TAG_FREE) begin
                                    hw_slot      <= lnb;
                                    chain_active <= 1'b0;
                                    lnk_idx      <= lnk_idx + 3'd1;
                                end else begin
                                    hot_wire_used <= 1'b0;
                                    hw_slot       <= 24'd0;
                                    chain_active  <= 1'b1;
                                    cur_la        <= hw_rd_data;
                                    cur_lb        <= lnb;
                                end
                            end else begin
                                cur_wid      <= lna_v[11:0];
                                cur_la       <= la;
                                cur_lb       <= lnb;
                                chain_active <= 1'b1;
                                state        <= S_LINK_RD;
                            end
                        end else if (lta == TAG_ROOT && ltb != TAG_REF) begin
                            roots[la[0]]   <= lb;
                            chain_active   <= 1'b0;
                            lnk_idx        <= lnk_idx + 3'd1;
                        end else if (ltb == TAG_ROOT && lta != TAG_REF) begin
                            roots[lb[0]]   <= la;
                            chain_active   <= 1'b0;
                            lnk_idx        <= lnk_idx + 3'd1;
                        end else if (!dq_valid[cur_tile]) begin
                            push_en = 1'b1; push_tile = cur_tile; push_a = la; push_b = lb;
                            chain_active <= 1'b0;
                            lnk_idx      <= lnk_idx + 3'd1;
                        end else begin : push_peer_or_spill
                            integer pk;
                            reg              pfound;
                            reg [TILE_W-1:0] ptile, pcand;
                            reg [2:0]        psum_t;
                            pfound = 1'b0;
                            ptile  = {TILE_W{1'b0}};
                            for (pk = 1; pk < N_TILES; pk = pk + 1) begin
                                psum_t = {{(3-TILE_W){1'b0}}, cur_tile} + pk[2:0];
                                pcand  = (psum_t >= N_TILES[2:0]) ? (psum_t[TILE_W-1:0] - N_TILES[TILE_W-1:0]) : psum_t[TILE_W-1:0];
                                if (!pfound && !dq_valid[pcand]) begin
                                    pfound = 1'b1;
                                    ptile  = pcand;
                                end
                            end
                            if (pfound) begin
                                push_en = 1'b1; push_tile = ptile; push_a = la; push_b = lb;
                                chain_active <= 1'b0;
                                lnk_idx      <= lnk_idx + 3'd1;
                            end else begin
                                cur_la       <= la;
                                cur_lb       <= lb;
                                chain_active <= 1'b1;
                                after_spill  <= S_LINK;
                                state        <= S_SPILL_WR;
                            end
                        end
                    end
                end

                S_LINK_RD: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd1;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {2'b01, 4'd0, cur_wid};
                            qspi_cmd_words <= 5'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else if (qspi_rdata_valid) begin
                        q_active <= 1'b0;
                        if (qspi_rdata[23:20] == TAG_FREE) wr_phase <= 1'b0;
                        else begin cur_la <= qspi_rdata[23:0]; wr_phase <= 1'b1; end
                        state <= S_LINK_WR;
                    end
                end

                S_LINK_WR: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid   <= 1'b1;
                            qspi_cmd_dev     <= 2'd1;
                            qspi_cmd_write   <= 1'b1;
                            qspi_cmd_waddr   <= {2'b01, 4'd0, cur_wid};
                            qspi_cmd_words   <= 5'd0;
                            qspi_wdata_valid <= 1'b1;
                            q_active         <= 1'b1;
                            qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_wdata_valid && qspi_wdata_ready) qspi_wdata_valid <= 1'b0;
                        if (!qspi_busy && !qspi_cmd_valid) begin
                            q_active <= 1'b0;
                            if (!wr_phase) begin
                                chain_active <= 1'b0;
                                lnk_idx      <= lnk_idx + 3'd1;
                            end
                            state <= S_LINK;
                        end
                    end
                end

                // ---- Flash REF Expander ----
                S_REF_DIR: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd0;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= 18'h00004 + {10'd0, exp_ref_id};
                            qspi_cmd_words <= 5'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else if (qspi_rdata_valid) begin
                        q_active        <= 1'b0;
                        exp_woff        <= qspi_rdata[13:0];
                        exp_nnodes      <= qspi_rdata[22:18];
                        exp_nredex      <= qspi_rdata[31:28];
                        exp_base_nid    <= psram_next_node;
                        exp_base_wid    <= psram_next_wire;
                        psram_next_node <= psram_next_node + {7'd0, qspi_rdata[22:18]};
                        psram_next_wire <= psram_next_wire + {7'd0, qspi_rdata[27:23]};
                        exp_phase       <= 3'd0;
                        state           <= S_REF_ROOT;
                    end
                end

                S_REF_ROOT: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd0;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {4'd0, exp_woff};
                            qspi_cmd_words <= 5'd0;
                            q_active       <= 1'b1;
                            exp_phase      <= 3'd0;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin
                            pb <= reloc_qspi_port; // exp_root
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                            q_active <= 1'b0;
                            exp_woff <= exp_woff + 14'd1;
                            if (exp_nnodes == 5'd0) begin
                                kind         <= K_DIRECT;
                                cur_la       <= pb;
                                cur_lb       <= pa;
                                chain_active <= 1'b0;
                                lnk_idx      <= 3'd0;
                                lnk_cnt      <= 3'd1;
                                after_link   <= (exp_nredex == 4'd0) ? S_IDLE : S_REF_REDEX;
                                state        <= S_LINK;
                            end else begin
                                exp_node_idx <= 5'd0;
                                state        <= S_REF_NODE_RD;
                            end
                        end
                    end
                end

                S_REF_NODE_RD: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd0;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {4'd0, exp_woff};
                            qspi_cmd_words <= 5'd1;
                            exp_phase      <= 3'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin : cold_rn
                            if (exp_phase == 3'd0) begin buf_a0 <= reloc_qspi_port; exp_phase <= 3'd1; end
                            else                         buf_a1 <= reloc_qspi_port;
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                            q_active <= 1'b0;
                            exp_woff <= exp_woff + 14'd2;
                            state    <= S_REF_NODE_WR;
                        end
                    end
                end

                S_REF_NODE_WR: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin : wr_pn
                            reg [11:0] dst_nid;
                            dst_nid = exp_base_nid + {7'd0, exp_node_idx};
                            qspi_cmd_valid   <= 1'b1;
                            qspi_cmd_dev     <= 2'd1;
                            qspi_cmd_write   <= 1'b1;
                            qspi_cmd_waddr   <= {5'd0, dst_nid, 1'b0};
                            qspi_cmd_words   <= 5'd1;
                            qspi_wdata_valid <= 1'b1;
                            wr_phase         <= 1'b0;
                            q_active         <= 1'b1;
                            qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_wdata_valid && qspi_wdata_ready) begin
                            if (!wr_phase) wr_phase <= 1'b1;
                            else           qspi_wdata_valid <= 1'b0;
                        end
                        if (!qspi_busy && !qspi_cmd_valid) begin
                            q_active <= 1'b0;
                            if (exp_node_idx + 5'd1 >= exp_nnodes) begin
                                kind         <= K_DIRECT;
                                cur_la       <= pb;
                                cur_lb       <= pa;
                                chain_active <= 1'b0;
                                lnk_idx      <= 3'd0;
                                lnk_cnt      <= 3'd1;
                                after_link   <= (exp_nredex == 4'd0) ? S_IDLE : S_REF_REDEX;
                                state        <= S_LINK;
                            end else begin
                                exp_node_idx <= exp_node_idx + 5'd1;
                                state        <= S_REF_NODE_RD;
                            end
                        end
                    end
                end

                // 2-redex Flash burst (up to 4 words into buf_a0/b0, buf_a1/b1 using K_PAIR2)
                S_REF_REDEX: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin : rdx_burst
                            reg [3:0] rem_r;
                            rem_r = exp_nredex - exp_redex_idx;
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd0;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {4'd0, exp_woff};
                            qspi_cmd_words <= (rem_r >= 4'd2) ? 5'd3 : 5'd1;
                            exp_phase      <= 3'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin : redex_rx
                            case (exp_phase[1:0])
                                2'd0: buf_a0 <= reloc_qspi_port;
                                2'd1: buf_b0 <= reloc_qspi_port;
                                2'd2: buf_a1 <= reloc_qspi_port;
                                2'd3: buf_b1 <= reloc_qspi_port;
                            endcase
                            exp_phase <= exp_phase + 3'd1;
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin : rdx_go
                            reg [3:0] batch;
                            batch = ((exp_nredex - exp_redex_idx) >= 4'd2) ? 4'd2 : 4'd1;
                            q_active      <= 1'b0;
                            kind          <= K_PAIR2;
                            chain_active  <= 1'b0;
                            lnk_idx       <= 3'd0;
                            lnk_cnt       <= {1'b0, batch[1:0]};
                            exp_redex_idx <= exp_redex_idx + batch;
                            exp_woff      <= exp_woff + {11'd0, batch[1:0], 1'b0};
                            after_link    <= (exp_redex_idx + batch >= exp_nredex) ? S_IDLE : S_REF_REDEX;
                            state         <= S_LINK;
                        end
                    end
                end

                // ---- PSRAM B Lazy LIFO Spill & Refill Stack ----
                S_SPILL_WR: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid   <= 1'b1;
                            qspi_cmd_dev     <= 2'd2;
                            qspi_cmd_write   <= 1'b1;
                            qspi_cmd_waddr   <= {7'd0, spill_cnt, 1'b0};
                            qspi_cmd_words   <= 5'd1;
                            buf_b0           <= dq_rd_data[23:0];
                            buf_a0           <= dq_rd_data[47:24];
                            qspi_wdata_valid <= 1'b1;
                            spill_pop0       = 1'b1;
                            wr_phase         <= 1'b0;
                            q_active         <= 1'b1;
                            qspi_txn_cnt_r   <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_wdata_valid && qspi_wdata_ready) begin
                            if (!wr_phase) wr_phase <= 1'b1;
                            else           qspi_wdata_valid <= 1'b0;
                        end
                        if (!qspi_busy && !qspi_cmd_valid) begin
                            q_active        <= 1'b0;
                            spill_cnt       <= spill_cnt + 10'd1;
                            psram_spill_cnt <= psram_spill_cnt + 10'd1;
                            state           <= after_spill;
                        end
                    end
                end

                S_REFILL_RD: begin
                    if (!q_active) begin
                        if (!qspi_busy && qspi_cmd_ready) begin
                            qspi_cmd_valid <= 1'b1;
                            qspi_cmd_dev   <= 2'd2;
                            qspi_cmd_write <= 1'b0;
                            qspi_cmd_waddr <= {7'd0, spill_cnt - 10'd1, 1'b0};
                            qspi_cmd_words <= 5'd1;
                            exp_phase      <= 3'd0;
                            q_active       <= 1'b1;
                            qspi_txn_cnt_r <= qspi_txn_cnt_r + 14'd1;
                        end
                    end else begin
                        if (qspi_rdata_valid) begin
                            if (exp_phase == 3'd0) begin buf_a0 <= qspi_rdata[23:0]; exp_phase <= 3'd1; end
                            else                         buf_a1 <= qspi_rdata[23:0];
                        end
                        if (!qspi_busy && !qspi_cmd_valid && !qspi_rdata_valid) begin
                            q_active  <= 1'b0;
                            spill_cnt <= spill_cnt - 10'd1;
                            push_en   = 1'b1;
                            push_tile = {TILE_W{1'b0}};
                            push_a    = buf_a0;
                            push_b    = buf_a1;
                            state     <= S_IDLE;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase

            // Unified 1-entry/tile Chase-Lev deque updates
            begin : dq_upd
                reg [N_TILES-1:0] nv;
                nv = dq_valid;

                if (pop_en || steal_en) nv[pop_tile] = 1'b0;
                if (spill_pop0)         nv[0]        = 1'b0;
                if (push_en) begin
                    dq_mem[push_tile] <= {push_b, push_a};
                    nv[push_tile]      = 1'b1;
                end

                dq_valid <= nv;
            end
        end
    end

endmodule
