// POE th_sch（一级发射调度器）行为模型：
// - 每拍从 READY 线程按 (pri, tid) 排序固定发射 ≤2 个：先按优先级（值小优先），
//   同优先级按 tid 小者优先；**全局取前 2 名**（无奇偶分组）
// - 同拍互斥：两 winner 若同为 ts 首个 loc burst（st=1 && lock_req）且 lock_id 相同，
//   只发高优先者，次者保持 READY 下拍重选（锁仅作用于一级发射阻塞）
// - 队列拆分：
//   * q0/q1（8 深槽池）仅存放 i/v burst，项含优先级，供 burst_sch 无头阻按优先级调度；
//   * c_task burst 解析出有效 c_task（c0/c1 + CSR.dma_c 判有效，CSR.cw 解析 tag/op），
//     存入独立 c_task 缓存（16 深槽池，按优先级调度；满则反压 c_task 发射）；
//     空 c_task burst（无有效任务）推 1 个占位 task，保证 dma 回 done（THM cur_ts 依赖）
// - 槽池无先后关系：调度器按优先级选任意有效槽，ack 逐槽清空
module poe_thsch #(
    parameter int MAX_THREADS = 64,
    parameter int QDEPTH = 8,
    parameter int CT_DEPTH = 16,
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
    // ---- CSR（解析 c_task 用）：dma_c 任务掩码 / cw 操作表 ----
    input logic [MAX_THREADS*8-1:0] csr_dma_c,
    input logic [MAX_THREADS*384-1:0] csr_cw,
    // ---- burst_sch 读侧：q0/q1 i/v 槽池（每槽一份扁平视图） ----
    output logic [QDEPTH-1:0] q0_vld,
    output logic [QDEPTH-1:0] q0_pre,
    output logic [QDEPTH*3-1:0] q0_pri,
    output logic [QDEPTH*6-1:0] q0_tid,
    output logic [QDEPTH*TS_ID_W-1:0] q0_ts,
    output logic [QDEPTH*4-1:0] q0_tidx,
    output logic [QDEPTH*BURST_W-1:0] q0_burst,
    input logic [QDEPTH-1:0] q0_ack, // 逐槽清空（本拍被调度的槽）
    output logic [QDEPTH-1:0] q1_vld,
    output logic [QDEPTH-1:0] q1_pre,
    output logic [QDEPTH*3-1:0] q1_pri,
    output logic [QDEPTH*6-1:0] q1_tid,
    output logic [QDEPTH*TS_ID_W-1:0] q1_ts,
    output logic [QDEPTH*4-1:0] q1_tidx,
    output logic [QDEPTH*BURST_W-1:0] q1_burst,
    input logic [QDEPTH-1:0] q1_ack,
    // ---- burst_sch 读侧：c_task 缓存（解析后的单 task 项） ----
    output logic [CT_DEPTH-1:0] ct_vld,
    output logic [CT_DEPTH*3-1:0] ct_pri,
    output logic [CT_DEPTH*6-1:0] ct_tid,
    output logic [CT_DEPTH*4-1:0] ct_tidx,
    output logic [CT_DEPTH*TS_ID_W-1:0] ct_ts,
    output logic [CT_DEPTH*3-1:0] ct_dma,
    output logic [CT_DEPTH*2-1:0] ct_tag,
    output logic [CT_DEPTH-1:0] ct_op,
    input logic [CT_DEPTH-1:0] ct_ack
    );

    import poe_types_pkg::*;

    // ---- i/v 槽项：{pre, pri(3), tid(6), ts(6), tidx(4), burst(38)} ----
    typedef struct packed {
        logic pre;
        logic [2:0] pri;
        logic [5:0] tid;
        logic [TS_ID_W-1:0] ts;
        logic [3:0] tidx;
        logic [BURST_W-1:0] burst;
    } iv_qitem_t;
    // ---- c_task 槽项：{pri(3), tid(6), tidx(4), ts(6), dma_id(3), tag(2), op(1)} ----
    typedef struct packed {
        logic [2:0] pri;
        logic [5:0] tid;
        logic [3:0] tidx;
        logic [TS_ID_W-1:0] ts; // 所属 ts 编号（DSE 发射需 ts==curts）
        logic [2:0] dma_id;
        logic [1:0] tag; // DSE（0..3）
        logic op; // 0=loc 1=free
    } ct_item_t;

    iv_qitem_t q0_mem [QDEPTH];
    logic [QDEPTH-1:0] q0_vld_r;
    iv_qitem_t q1_mem [QDEPTH];
    logic [QDEPTH-1:0] q1_vld_r;
    ct_item_t ct_mem [CT_DEPTH];
    logic [CT_DEPTH-1:0] ct_vld_r;

    logic iss0, iss1;
    int tid0, tid1;
    logic [2:0] pri0, pri1;
    burst_iv_t b0, b1;

    // 全局取 (pri, tid) 最小的两个 READY 线程（无奇偶分组）；
    // 第二名胜出者排除与第一名同锁 id 的 loc burst（st=1 && lock_req）
    always_comb begin
        automatic burst_iv_t rb0;
        automatic logic lock0_eff;
        iss0 = 1'b0; tid0 = 0;
        iss1 = 1'b0; tid1 = 0;
        pri0 = 3'd7; pri1 = 3'd7;
        // 第一遍：全局第一名
        for (int s = 0; s < MAX_THREADS; s++)
            if (ready_mask[s]) begin
                automatic logic [2:0] p = ready_pri[s*3 +: 3];
                if (p < pri0 || (p == pri0 && s < tid0) || !iss0) begin
                    iss0 = 1'b1; tid0 = s; pri0 = p;
                end
            end
        rb0 = ready_burst[tid0*BURST_W +: BURST_W];
        lock0_eff = iss0 && rb0.st && rb0.lock_req;
        // 第二遍：全局第二名（排除 tid0 与同锁 id 冲突线程）
        for (int s = 0; s < MAX_THREADS; s++)
            if (ready_mask[s] && s != tid0) begin
                automatic logic [2:0] p = ready_pri[s*3 +: 3];
                automatic burst_iv_t rb = ready_burst[s*BURST_W +: BURST_W];
                if (lock0_eff && rb.st && rb.lock_req && rb.lock_id == rb0.lock_id)
                    continue; // 同拍不能同时发射同锁 id 的 loc burst
                if (p < pri1 || (p == pri1 && s < tid1) || !iss1) begin
                    iss1 = 1'b1; tid1 = s; pri1 = p;
                end
            end
    end

    assign b0 = ready_burst[tid0*BURST_W +: BURST_W];
    assign b1 = ready_burst[tid1*BURST_W +: BURST_W];

    // ---- 空闲槽计数（ct 16 槽需 5bit，4bit 会回绕成 0） ----
    logic [4:0] free_q0, free_q1, free_ct;
    always_comb begin
        free_q0 = 5'd0; free_q1 = 5'd0; free_ct = 5'd0;
        for (int i = 0; i < QDEPTH; i++) begin
            if (!q0_vld_r[i]) free_q0 = free_q0 + 1'b1;
            if (!q1_vld_r[i]) free_q1 = free_q1 + 1'b1;
        end
        for (int i = 0; i < CT_DEPTH; i++)
            if (!ct_vld_r[i]) free_ct = free_ct + 1'b1;
    end

    // ---- c_task burst 需占用的缓存槽数（无有效任务时按 1 计，保证 done 路径） ----
    logic [1:0] need0, need1;
    always_comb begin
        need0 = 2'd0; need1 = 2'd0;
        if (iss0 && b0.burst_type) begin
            automatic burst_c_t cb = ready_burst[tid0*BURST_W +: BURST_W];
            automatic logic [7:0] dc = csr_dma_c[tid0*8 +: 8];
            if (cb.c0 && dc[cb.dma_id0]) need0 = need0 + 2'd1;
            if (cb.vld_cu && cb.c1 && dc[cb.dma_id1]) need0 = need0 + 2'd1;
            if (need0 == 2'd0) need0 = 2'd1;
        end
        if (iss1 && b1.burst_type) begin
            automatic burst_c_t cb = ready_burst[tid1*BURST_W +: BURST_W];
            automatic logic [7:0] dc = csr_dma_c[tid1*8 +: 8];
            if (cb.c0 && dc[cb.dma_id0]) need1 = need1 + 2'd1;
            if (cb.vld_cu && cb.c1 && dc[cb.dma_id1]) need1 = need1 + 2'd1;
            if (need1 == 2'd0) need1 = 2'd1;
        end
    end

    // ---- 发射使能：i/v → 对应槽池有空；c_task → c_task 缓存有足够空间 ----
    assign iss_vld0 = iss0 && (b0.burst_type ? (free_ct >= need0) : (free_q0 != 5'd0));
    assign iss_tid0 = tid0[5:0];
    assign iss_vld1 = iss1 && (b1.burst_type ? (free_ct >= need0 + need1) : (free_q1 != 5'd0));
    assign iss_tid1 = tid1[5:0];

    // ---- 找第 n 个空闲槽（跳过 excl 中已分配索引） ----
    function automatic int q_free_idx(input int n, input logic [7:0] excl,
                                      input logic [7:0] vld);
        automatic int c = 0;
        for (int i = 0; i < QDEPTH; i++)
            if (!vld[i] && !excl[i]) begin
                if (c == n) return i;
                c++;
            end
        return -1;
    endfunction
    function automatic int ct_free_idx(input int n, input logic [15:0] excl);
        automatic int c = 0;
        for (int i = 0; i < CT_DEPTH; i++)
            if (!ct_vld_r[i] && !excl[i]) begin
                if (c == n) return i;
                c++;
            end
        return -1;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < QDEPTH; i++) begin
                q0_vld_r[i] <= 1'b0;
                q1_vld_r[i] <= 1'b0;
            end
            for (int i = 0; i < CT_DEPTH; i++) ct_vld_r[i] <= 1'b0;
        end else begin
            automatic logic [15:0] ct_excl = 16'b0;
            // ---- 清槽（burst_sch 本拍调度的槽） ----
            for (int i = 0; i < QDEPTH; i++) begin
                if (q0_ack[i]) q0_vld_r[i] <= 1'b0;
                if (q1_ack[i]) q1_vld_r[i] <= 1'b0;
            end
            for (int i = 0; i < CT_DEPTH; i++)
                if (ct_ack[i]) ct_vld_r[i] <= 1'b0;
            // ---- winner0：i/v → q0；c_task → 解析入 c_task 缓存 ----
            if (iss_vld0) begin
                if (!b0.burst_type) begin
                    automatic int idx = q_free_idx(0, 8'b0, q0_vld_r);
                    if (idx >= 0) begin
                        q0_mem[idx] <= {1'b0,
                        ready_pri[tid0*3 +: 3],
                        tid0[5:0],
                        ready_burst_ts[tid0*TS_ID_W +: TS_ID_W],
                        ready_burst_tidx[tid0*4 +: 4],
                        ready_burst[tid0*BURST_W +: BURST_W]};
                        q0_vld_r[idx] <= 1'b1;
                    end
                end else begin
                    automatic burst_c_t cb = ready_burst[tid0*BURST_W +: BURST_W];
                    automatic logic [7:0] dc = csr_dma_c[tid0*8 +: 8];
                    automatic logic t0 = cb.c0 && dc[cb.dma_id0];
                    automatic logic t1 = cb.vld_cu && cb.c1 && dc[cb.dma_id1];
                    automatic int idx;
                    if (t0) begin
                        automatic cw_entry_t ce = csr_cw[tid0*384 +: 384][cb.dma_id0*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid0*3 +: 3], tid0[5:0],
                            ready_burst_tidx[tid0*4 +: 4],
                            ready_burst_ts[tid0*TS_ID_W +: TS_ID_W], cb.dma_id0,
                            ce.tag[1:0], ce.op_type};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                    if (t1) begin
                        automatic cw_entry_t ce = csr_cw[tid0*384 +: 384][cb.dma_id1*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid0*3 +: 3], tid0[5:0],
                            ready_burst_tidx[tid0*4 +: 4],
                            ready_burst_ts[tid0*TS_ID_W +: TS_ID_W], cb.dma_id1,
                            ce.tag[1:0], ce.op_type};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                    if (!t0 && !t1) begin // 空 c_task：占位 task（保 done 路径）
                        automatic cw_entry_t ce = csr_cw[tid0*384 +: 384][cb.dma_id0*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid0*3 +: 3], tid0[5:0],
                            ready_burst_tidx[tid0*4 +: 4],
                            ready_burst_ts[tid0*TS_ID_W +: TS_ID_W], cb.dma_id0,
                            2'd0, 1'b0};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                end
            end
            // ---- winner1：i/v → q1；c_task → c_task 缓存（继续 winner0 的 excl） ----
            if (iss_vld1) begin
                if (!b1.burst_type) begin
                    automatic int idx = q_free_idx(0, 8'b0, q1_vld_r);
                    if (idx >= 0) begin
                        q1_mem[idx] <= {1'b0,
                        ready_pri[tid1*3 +: 3],
                        tid1[5:0],
                        ready_burst_ts[tid1*TS_ID_W +: TS_ID_W],
                        ready_burst_tidx[tid1*4 +: 4],
                        ready_burst[tid1*BURST_W +: BURST_W]};
                        q1_vld_r[idx] <= 1'b1;
                    end
                end else begin
                    automatic burst_c_t cb = ready_burst[tid1*BURST_W +: BURST_W];
                    automatic logic [7:0] dc = csr_dma_c[tid1*8 +: 8];
                    automatic logic t0 = cb.c0 && dc[cb.dma_id0];
                    automatic logic t1 = cb.vld_cu && cb.c1 && dc[cb.dma_id1];
                    automatic int idx;
                    if (t0) begin
                        automatic cw_entry_t ce = csr_cw[tid1*384 +: 384][cb.dma_id0*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid1*3 +: 3], tid1[5:0],
                            ready_burst_tidx[tid1*4 +: 4],
                            ready_burst_ts[tid1*TS_ID_W +: TS_ID_W], cb.dma_id0,
                            ce.tag[1:0], ce.op_type};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                    if (t1) begin
                        automatic cw_entry_t ce = csr_cw[tid1*384 +: 384][cb.dma_id1*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid1*3 +: 3], tid1[5:0],
                            ready_burst_tidx[tid1*4 +: 4],
                            ready_burst_ts[tid1*TS_ID_W +: TS_ID_W], cb.dma_id1,
                            ce.tag[1:0], ce.op_type};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                    if (!t0 && !t1) begin
                        automatic cw_entry_t ce = csr_cw[tid1*384 +: 384][cb.dma_id0*48 +: 48];
                        idx = ct_free_idx(0, ct_excl);
                        if (idx >= 0) begin
                            ct_mem[idx] <= {ready_pri[tid1*3 +: 3], tid1[5:0],
                            ready_burst_tidx[tid1*4 +: 4],
                            ready_burst_ts[tid1*TS_ID_W +: TS_ID_W], cb.dma_id0,
                            2'd0, 1'b0};
                            ct_vld_r[idx] <= 1'b1;
                            ct_excl[idx] = 1'b1;
                        end
                    end
                end
            end
        end
    end

    // ---- 扁平导出：burst_sch 按优先级调度任意有效槽 ----
    always_comb begin
        for (int i = 0; i < QDEPTH; i++) begin
            q0_vld[i] = q0_vld_r[i];
            q0_pre[i] = q0_mem[i].pre;
            q0_pri[i*3 +: 3] = q0_mem[i].pri;
            q0_tid[i*6 +: 6] = q0_mem[i].tid;
            q0_ts[i*TS_ID_W +: TS_ID_W] = q0_mem[i].ts;
            q0_tidx[i*4 +: 4] = q0_mem[i].tidx;
            q0_burst[i*BURST_W +: BURST_W] = q0_mem[i].burst;
            q1_vld[i] = q1_vld_r[i];
            q1_pre[i] = q1_mem[i].pre;
            q1_pri[i*3 +: 3] = q1_mem[i].pri;
            q1_tid[i*6 +: 6] = q1_mem[i].tid;
            q1_ts[i*TS_ID_W +: TS_ID_W] = q1_mem[i].ts;
            q1_tidx[i*4 +: 4] = q1_mem[i].tidx;
            q1_burst[i*BURST_W +: BURST_W] = q1_mem[i].burst;
        end
        for (int i = 0; i < CT_DEPTH; i++) begin
            ct_vld[i] = ct_vld_r[i];
            ct_pri[i*3 +: 3] = ct_mem[i].pri;
            ct_tid[i*6 +: 6] = ct_mem[i].tid;
            ct_tidx[i*4 +: 4] = ct_mem[i].tidx;
            ct_ts[i*TS_ID_W +: TS_ID_W] = ct_mem[i].ts;
            ct_dma[i*3 +: 3] = ct_mem[i].dma_id;
            ct_tag[i*2 +: 2] = ct_mem[i].tag;
            ct_op[i] = ct_mem[i].op;
        end
    end
endmodule
