// POE burst_sch（二级发射调度器）：
// - i/v：q0/q1 两个槽池各自独立调度（无头阻，按优先级选任意有效槽），
//   每拍 q0→EU0、q1→EU1 各最多 1 个；可发射条件：
//   ① ts == 线程当前 cur_ts（pre 槽跳过）
//   ② 不满足"O 窗反压 且 burst 涉及 O 窗操作"
// - c_task：4 个独立 DSE 调度器（DSE 0..3，DSE==tag），各自从 c_task 缓存按优先级
//   选 tag 匹配的最高优先项，每拍最多 4 个（每 DSE 1 个），**发射即完成**：
//   done 由二级发射直接返回 THM（寄存 1 拍）；发射的 c_task 推入 dma_ctrl
//   对应 DSE 的 c_task FIFO，**FIFO 满时反压该 DSE 发射**（槽保留，下拍重选）；
//   c_task 仍须满足 ts == 线程当前 cur_ts（依赖 burst 分处不同 ts，保证 loc/free
//   等先后顺序）；"无先后关系"指槽池无 FIFO/头阻，按优先级选任意槽
// - pre_read 预读：8 深缓存暂存（占位，接收即吸收，预读语义待细化）
module poe_burstsch #(
    parameter int MAX_THREADS = 64,
    parameter int TS_ID_W = 6,
    parameter int QDEPTH = 8,
    parameter int CT_DEPTH = 16
    ) (
    input logic clk,
    input logic rst_n,
    // ---- th_sch：q0/q1 i/v 槽池（逐槽扁平视图） ----
    input logic [QDEPTH-1:0] q0_vld,
    input logic [QDEPTH-1:0] q0_pre,
    input logic [QDEPTH*3-1:0] q0_pri,
    input logic [QDEPTH*6-1:0] q0_tid,
    input logic [QDEPTH*TS_ID_W-1:0] q0_ts,
    input logic [QDEPTH*4-1:0] q0_tidx,
    input logic [QDEPTH*BURST_W-1:0] q0_burst,
    output logic [QDEPTH-1:0] q0_ack,
    input logic [QDEPTH-1:0] q1_vld,
    input logic [QDEPTH-1:0] q1_pre,
    input logic [QDEPTH*3-1:0] q1_pri,
    input logic [QDEPTH*6-1:0] q1_tid,
    input logic [QDEPTH*TS_ID_W-1:0] q1_ts,
    input logic [QDEPTH*4-1:0] q1_tidx,
    input logic [QDEPTH*BURST_W-1:0] q1_burst,
    output logic [QDEPTH-1:0] q1_ack,
    // ---- th_sch：c_task 缓存（解析后的单 task 项） ----
    input logic [CT_DEPTH-1:0] ct_vld,
    input logic [CT_DEPTH*3-1:0] ct_pri,
    input logic [CT_DEPTH*6-1:0] ct_tid,
    input logic [CT_DEPTH*4-1:0] ct_tidx,
    input logic [CT_DEPTH*TS_ID_W-1:0] ct_ts,
    input logic [CT_DEPTH*3-1:0] ct_dma,
    input logic [CT_DEPTH*2-1:0] ct_tag,
    input logic [CT_DEPTH-1:0] ct_op,
    output logic [CT_DEPTH-1:0] ct_ack,
    // ---- 线程状态 / O 窗反压 ----
    input logic [MAX_THREADS*TS_ID_W-1:0] thread_curts,
    input logic owin_bp,
    // ---- i/v 发射（→ EU0/EU1） ----
    output logic emit_eu_vld0,
    output logic [5:0] emit_eu_tid0,
    output logic [3:0] emit_eu_tidx0,
    output logic [BURST_W-1:0] emit_eu_burst0,
    input logic eu0_ack,
    output logic emit_eu_vld1,
    output logic [5:0] emit_eu_tid1,
    output logic [3:0] emit_eu_tidx1,
    output logic [BURST_W-1:0] emit_eu_burst1,
    input logic eu1_ack,
    // ---- c_task 发射（每 DSE 1 个，发射即完成：done 由二级发射直接返回） ----
    output logic [3:0] emit_dma_vld,
    output logic [23:0] emit_dma_tid,
    output logic [15:0] emit_dma_tidx,
    output logic [11:0] emit_dma_dma_id,
    output logic [7:0] emit_dma_tag,
    output logic [3:0] emit_dma_op,
    input logic [3:0] dma_full, // dma_ctrl c_task FIFO 满（满则反压本 DSE）
    // ---- c_task done（发射后 1 拍返回 THM，无需执行单元） ----
    output logic [3:0] dma_done_vld,
    output logic [23:0] dma_done_tid,
    output logic [15:0] dma_done_tidx,
    // ---- pre_read 预读接口（THM → 缓存；占位吸收） ----
    input logic [3:0] pre_vld,
    input logic [23:0] pre_tid,
    input logic [79:0] pre_dma_addr,
    input logic [3:0] pre_op,
    output logic pre_buf_rdy
    );

    import poe_types_pkg::*;

    // ---- pre_read 预读缓存（8 深 × {v, tid, dma_addr, op}；占位：吸收不执行） ----
    localparam int PRE_BUF_DEPTH = 8;
    logic [27:0] pre_buf [PRE_BUF_DEPTH];
    logic [2:0] pre_head, pre_tail;
    logic [3:0] pre_cnt;
    logic [1:0] pre_out_n; // 本拍吸收组数（0..4）

    assign pre_buf_rdy = (pre_cnt <= PRE_BUF_DEPTH - 4); // 需容纳每拍最多 4 组
    always_comb begin
        pre_out_n = 2'd0;
        for (int k = 0; k < 4; k++)
            if (k < pre_cnt) pre_out_n = k[1:0] + 1'b1;
    end
    logic [2:0] pre_wr_n;
    always_comb begin
        pre_wr_n = 3'd0;
        for (int k = 0; k < 4; k++)
            if (pre_vld[k]) pre_wr_n = pre_wr_n + 3'd1;
    end
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < PRE_BUF_DEPTH; i++) pre_buf[i] <= '0;
            pre_head <= '0;
            pre_tail <= '0;
            pre_cnt <= '0;
        end else begin
            if (pre_wr_n != 3'd0) begin
                if (pre_vld[0])
                    pre_buf[pre_tail] <= {1'b1, pre_tid[5:0],
                                          pre_dma_addr[19:0], pre_op[0]};
                if (pre_vld[1])
                    pre_buf[pre_tail + 1'b1] <= {1'b1, pre_tid[11:6],
                                                 pre_dma_addr[39:20], pre_op[1]};
                if (pre_vld[2])
                    pre_buf[pre_tail + 2'd2] <= {1'b1, pre_tid[17:12],
                                                 pre_dma_addr[59:40], pre_op[2]};
                if (pre_vld[3])
                    pre_buf[pre_tail + 2'd3] <= {1'b1, pre_tid[23:18],
                                                 pre_dma_addr[79:60], pre_op[3]};
                pre_tail <= pre_tail + pre_wr_n;
            end
            // 吸收出队（占位：不执行、不回 done）
            if (pre_out_n != 2'd0) pre_head <= pre_head + pre_out_n;
            pre_cnt <= pre_cnt + pre_wr_n - {2'd0, pre_out_n};
        end
    end

    // ---- q0 → EU0：q0 槽池内按优先级选最高可发射（无头阻） ----
    int pick0;
    logic emit0;
    always_comb begin
        automatic logic [2:0] best_pri = 3'd7;
        pick0 = -1;
        emit0 = 1'b0;
        for (int i = 0; i < QDEPTH; i++)
            if (q0_vld[i]) begin
                automatic burst_iv_t b = q0_burst[i*BURST_W +: BURST_W];
                automatic logic [2:0] p = q0_pri[i*3 +: 3];
                if ((q0_pre[i] ||
                     (q0_ts[i*TS_ID_W +: TS_ID_W] ==
                      thread_curts[q0_tid[i*6 +: 6]*TS_ID_W +: TS_ID_W])) &&
                    !(owin_bp && b.tr) &&
                    (p < best_pri || (p == best_pri && (pick0 < 0 || i < pick0)) ||
                     pick0 < 0)) begin
                    best_pri = p;
                    pick0 = i;
                end
            end
        if (pick0 >= 0 && eu0_ack) emit0 = 1'b1;
    end
    assign q0_ack = emit0 ? (8'd1 << pick0) : 8'b0;
    assign emit_eu_vld0 = emit0;
    assign emit_eu_tid0 = emit0 ? q0_tid[pick0*6 +: 6] : 6'd0;
    assign emit_eu_tidx0 = emit0 ? q0_tidx[pick0*4 +: 4] : 4'd0;
    assign emit_eu_burst0 = emit0 ? q0_burst[pick0*BURST_W +: BURST_W] : '0;

    // ---- q1 → EU1：同上 ----
    int pick1;
    logic emit1;
    always_comb begin
        automatic logic [2:0] best_pri = 3'd7;
        pick1 = -1;
        emit1 = 1'b0;
        for (int i = 0; i < QDEPTH; i++)
            if (q1_vld[i]) begin
                automatic burst_iv_t b = q1_burst[i*BURST_W +: BURST_W];
                automatic logic [2:0] p = q1_pri[i*3 +: 3];
                if ((q1_pre[i] ||
                     (q1_ts[i*TS_ID_W +: TS_ID_W] ==
                      thread_curts[q1_tid[i*6 +: 6]*TS_ID_W +: TS_ID_W])) &&
                    !(owin_bp && b.tr) &&
                    (p < best_pri || (p == best_pri && (pick1 < 0 || i < pick1)) ||
                     pick1 < 0)) begin
                    best_pri = p;
                    pick1 = i;
                end
            end
        if (pick1 >= 0 && eu1_ack) emit1 = 1'b1;
    end
    assign q1_ack = emit1 ? (8'd1 << pick1) : 8'b0;
    assign emit_eu_vld1 = emit1;
    assign emit_eu_tid1 = emit1 ? q1_tid[pick1*6 +: 6] : 6'd0;
    assign emit_eu_tidx1 = emit1 ? q1_tidx[pick1*4 +: 4] : 4'd0;
    assign emit_eu_burst1 = emit1 ? q1_burst[pick1*BURST_W +: BURST_W] : '0;

    // ---- 4 个 DSE 调度器：各选 tag 匹配的最高优先 c_task（无头阻，互不冲突） ----
    logic [3:0] ct_pick_vld;
    int ct_pick [4];
    always_comb begin
        for (int d = 0; d < 4; d++) begin
            automatic logic [2:0] best_pri = 3'd7;
            ct_pick_vld[d] = 1'b0;
            ct_pick[d] = -1;
            for (int i = 0; i < CT_DEPTH; i++)
                if (ct_vld[i] &&
                    (ct_tag[i*2 +: 2] == d[1:0]) &&
                    (ct_ts[i*TS_ID_W +: TS_ID_W] ==
                     thread_curts[ct_tid[i*6 +: 6]*TS_ID_W +: TS_ID_W])) begin
                    automatic logic [2:0] p = ct_pri[i*3 +: 3];
                    if (p < best_pri ||
                        (p == best_pri && (ct_pick[d] < 0 || i < ct_pick[d])) ||
                        ct_pick[d] < 0) begin
                        best_pri = p;
                        ct_pick[d] = i;
                        ct_pick_vld[d] = 1'b1;
                    end
                end
        end
    end

    // 发射使能：DSE 调度器选中且对应 c_task FIFO 未满（满则反压，槽保留）
    logic [3:0] emit_dma;
    always_comb begin
        for (int d = 0; d < 4; d++) begin
            emit_dma[d] = ct_pick_vld[d] && !dma_full[d];
            emit_dma_vld[d] = emit_dma[d];
            emit_dma_tid[d*6 +: 6] = emit_dma[d] ? ct_tid[ct_pick[d]*6 +: 6] : 6'd0;
            emit_dma_tidx[d*4 +: 4] = emit_dma[d] ? ct_tidx[ct_pick[d]*4 +: 4] : 4'd0;
            emit_dma_dma_id[d*3 +: 3] = emit_dma[d] ? ct_dma[ct_pick[d]*3 +: 3] : 3'd0;
            emit_dma_tag[d*2 +: 2] = emit_dma[d] ? ct_tag[ct_pick[d]*2 +: 2] : 2'd0;
            emit_dma_op[d] = emit_dma[d] ? ct_op[ct_pick[d]] : 1'b0;
        end
        // 逐槽 ack（tag 唯一，最多一个 DSE 命中）
        for (int i = 0; i < CT_DEPTH; i++) begin
            ct_ack[i] = 1'b0;
            for (int d = 0; d < 4; d++)
                if (emit_dma[d] && (ct_pick[d] == i)) ct_ack[i] = 1'b1;
        end
    end

    // ---- c_task done：发射寄存 1 拍后返回（发射即完成） ----
    logic [3:0] dma_done_vld_r;
    logic [23:0] dma_done_tid_r;
    logic [15:0] dma_done_tidx_r;
    assign dma_done_vld = dma_done_vld_r;
    assign dma_done_tid = dma_done_tid_r;
    assign dma_done_tidx = dma_done_tidx_r;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_done_vld_r <= '0;
            dma_done_tid_r <= '0;
            dma_done_tidx_r <= '0;
        end else begin
            dma_done_vld_r <= emit_dma;
            dma_done_tid_r <= emit_dma_tid;
            dma_done_tidx_r <= emit_dma_tidx;
        end
    end
endmodule
