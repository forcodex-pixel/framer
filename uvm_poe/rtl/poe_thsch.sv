// POE th_sch（一级发射调度器）行为模型：
// - 每拍从 READY 线程按 (pri, tid) 排序固定发射 ≤2 个：先按优先级（值小优先），
//   同优先级按 tid 小者优先；**全局取前 2 名**（无奇偶分组）
// - 路由：winner0 → q0、winner1 → q1（各受队列满反压），不做线程固定 lane；
//   线程内 burst 顺序由 ts 机制保证：同一配对（同 tag/同锁）的 loc/free 有先后
//   依赖，分处不同 ts，burst_sch 按 `ts == cur_ts` 发射，free 必然在 loc 完成后
//   才放行；不同锁/不同 tag 的 loc/free 相互独立，无先后要求
// - 发射的 burst 写入 2 个 burst 队列（深度 QDEPTH），队列项携带：
//   {pre, burst(38b), burst_ts, ts_idx(4b), th_id(6b)}（ts_idx 供 CU/dma 完成反馈归属）
//   - burst：THM ready_burst（bs_pc 索引，38bit；burst_type 区分 i/v 与 c_task 字段）
//   - burst_ts：发射时刻该 burst 所属 ts（THM ready_burst_ts）
//   - pre：预留插队标志（当前 pre_read 走 burst_sch 独立预读缓存，不经本队列）
// - 队列未满才发射（写侧反压）；读侧 vld/ack 接口供 burst_sch 取
module poe_thsch #(
    parameter int MAX_THREADS = 64,
    parameter int QDEPTH = 8,
    parameter int TS_ID_W = 6
    ) (

    input logic clk,
    input logic rst_n,
    // ---- THM 侧 ----
    input logic [MAX_THREADS-1:0] ready_mask,
    input logic [MAX_THREADS*3-1:0] ready_pri,
    input logic [MAX_THREADS*TS_ID_W-1:0] ready_burst_ts,
    input logic [MAX_THREADS*4-1:0] ready_burst_tidx, // 每线程当前发射 burst 的 ts 序号
    input logic [MAX_THREADS*BURST_W-1:0] ready_burst,
    output logic iss_vld0,
    output logic [5:0] iss_tid0,
    output logic iss_vld1,
    output logic [5:0] iss_tid1,
    // ---- burst_sch 侧（2 个 burst 队列读侧） ----
    output logic q0_vld,
    output logic [5:0] q0_tid,
    output logic [TS_ID_W-1:0] q0_ts,
    output logic [3:0] q0_tidx,
    output logic [BURST_W-1:0] q0_burst,
    output logic q0_pre,
    input logic q0_ack,
    output logic q1_vld,
    output logic [5:0] q1_tid,
    output logic [TS_ID_W-1:0] q1_ts,
    output logic [3:0] q1_tidx,
    output logic [BURST_W-1:0] q1_burst,
    output logic q1_pre,
    input logic q1_ack
    );

    import poe_types_pkg::*;

    localparam int PTR_W = $clog2(QDEPTH);
    // 队列项 {pre, burst, burst_ts, th_id}
    localparam int QW = 1 + BURST_W + TS_ID_W + 4 + 6; // {pre, burst, burst_ts, ts_idx, th_id}
    logic [QW-1:0] q0_mem [QDEPTH];
    logic [PTR_W-1:0] q0_head, q0_tail;
    logic [PTR_W:0] q0_cnt;
    logic [QW-1:0] q1_mem [QDEPTH];
    logic [PTR_W-1:0] q1_head, q1_tail;
    logic [PTR_W:0] q1_cnt;

    logic iss0, iss1;
    int tid0, tid1;
    logic [2:0] pri0, pri1;
    // 全局取 (pri, tid) 最小的两个 READY 线程（无奇偶分组）
    always_comb begin
        iss0 = 1'b0; tid0 = 0;
        iss1 = 1'b0; tid1 = 0;
        pri0 = 3'd7; pri1 = 3'd7;
        for (int s = 0; s < MAX_THREADS; s++)
            if (ready_mask[s]) begin
                automatic logic [2:0] p = ready_pri[s*3 +: 3];
                if (p < pri0 || (p == pri0 && s < tid0) || !iss0) begin
                    // 原第一名降为第二名
                    iss1 = iss0; tid1 = tid0; pri1 = pri0;
                    iss0 = 1'b1; tid0 = s; pri0 = p;
                end else if (p < pri1 || (p == pri1 && s < tid1) || !iss1) begin
                    iss1 = 1'b1; tid1 = s; pri1 = p;
                end
            end
    end

    assign q0_vld = (q0_cnt != 0);
    assign q1_vld = (q1_cnt != 0);
    assign q0_tid = q0_mem[q0_head][5:0];
    assign q0_ts = q0_mem[q0_head][TS_ID_W+9:TS_ID_W+4]; // burst_ts（tidx 插入后位段）
    assign q0_tidx = q0_mem[q0_head][TS_ID_W+3:TS_ID_W];
    assign q0_burst = q0_mem[q0_head][QW-2:TS_ID_W+10];
    assign q0_pre = q0_mem[q0_head][QW-1];
    assign q1_tid = q1_mem[q1_head][5:0];
    assign q1_ts = q1_mem[q1_head][TS_ID_W+9:TS_ID_W+4];
    assign q1_tidx = q1_mem[q1_head][TS_ID_W+3:TS_ID_W];
    assign q1_burst = q1_mem[q1_head][QW-2:TS_ID_W+10];
    assign q1_pre = q1_mem[q1_head][QW-1];

    // 发射使能：各目标队列未满（winner0→q0、winner1→q1）
    assign iss_vld0 = iss0 && (q0_cnt != QDEPTH);
    assign iss_tid0 = tid0[5:0];
    assign iss_vld1 = iss1 && (q1_cnt != QDEPTH);
    assign iss_tid1 = tid1[5:0];

    logic q0_wr, q1_wr;
    assign q0_wr = iss_vld0; // winner0 → q0
    assign q1_wr = iss_vld1; // winner1 → q1

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q0_head <= '0; q0_tail <= '0; q0_cnt <= '0;
            q1_head <= '0; q1_tail <= '0; q1_cnt <= '0;
        end else begin
            // 发射写队列（一级发射）
            if (q0_wr) begin
                q0_mem[q0_tail] <= {1'b0,
                ready_burst[tid0*BURST_W +: BURST_W],
                ready_burst_ts[tid0*TS_ID_W +: TS_ID_W],
                ready_burst_tidx[tid0*4 +: 4], tid0[5:0]};
                q0_tail <= (q0_tail == QDEPTH-1) ? '0 : q0_tail + 1'b1;
            end
            if (q1_wr) begin
                q1_mem[q1_tail] <= {1'b0,
                ready_burst[tid1*BURST_W +: BURST_W],
                ready_burst_ts[tid1*TS_ID_W +: TS_ID_W],
                ready_burst_tidx[tid1*4 +: 4], tid1[5:0]};
                q1_tail <= (q1_tail == QDEPTH-1) ? '0 : q1_tail + 1'b1;
            end
            // burst_sch 读
            if (q0_ack && q0_vld) begin
                q0_head <= (q0_head == QDEPTH-1) ? '0 : q0_head + 1'b1;
            end
            if (q1_ack && q1_vld) begin
                q1_head <= (q1_head == QDEPTH-1) ? '0 : q1_head + 1'b1;
            end
            // 计数：写 +1 / 读 -1（同拍写读净 0）
            if (q0_wr && !(q0_ack && q0_vld)) q0_cnt <= q0_cnt + 1'b1;
            else if (!q0_wr && (q0_ack && q0_vld)) q0_cnt <= q0_cnt - 1'b1;
            if (q1_wr && !(q1_ack && q1_vld)) q1_cnt <= q1_cnt + 1'b1;
            else if (!q1_wr && (q1_ack && q1_vld)) q1_cnt <= q1_cnt - 1'b1;
        end
    end
endmodule
