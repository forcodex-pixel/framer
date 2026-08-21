// ============================================================================
// POE dma_ctrl 行为模型：c_task（loc/free）+ C 窗资源管理 + SMC/RBA 模型
// 结构（对应《dma_ctrl设计方案.md》）：
// - 接收 burst_sch 二级发射的 c_task burst（每拍 ≤1，含 ≤2 个 c_task）；
//   按 c0/c1 + CSR.dma_c 判有效任务，按 dma_id 查 CSR.cw 得 op_type（loc/free）
//   与 smc 地址（tag）；同拍两个 free 同地址只执行端口小者。
// - C 窗分独享/共享两类（cw 8 项固定映射：dma_id 0..3 → 独享，4..7 → 共享）：
//   独享 = MAX_THREADS×4，每线程固定位置（全局号 = tid×4 + dma_id[1:0]），
//   申请/释放不经过 FIFO；共享 = C_WND_SHR_NUM(256)，由资源管理 FIFO 分配。
// - THM 线程级互斥锁（16 个独立锁）保证同一 smc 地址同一时刻只有一个线程执行，
//   因此**不再需要 C 窗扫描查重/指令预存/转交**：loc 直接申请、free 直接释放。
// - loc（0）：按条目类型申请（独享固定位置 / 共享 FIFO 出队，共享单线程上限 4）、写 C 窗条目
//   （tag/c_line/d/o/r/cnt/ind）、更新 CSR.cw（o=1、c_line_num、start_ts、occ_ts）、
//   经 RBA 读 SMC 数据回填 c_line 并置 r=1。
// - free（1）：RBA 把 C 窗 c_line 写回 SMC[tag] → C 窗条目 o=0（共享资源号入 FIFO、
//   th_res_n_r-1；独享固定位置不入 FIFO）、cw.o=0。
// - 资源生命周期完全由 c_task 控制（lock/free 成对出现），线程结束不兜底归还；
//   dma_done 仅表示 burst 执行完成（THM cur_ts 推进用），不归还资源。
// - pre 插队 burst：不占资源、不执行、不回 done（占位，预读语义待细化）。
// ============================================================================
module poe_dma_ctrl #(
    parameter int MAX_THREADS = 64,
    parameter int TS_ID_W = 6,
    parameter int C_WND_SHR_NUM = 256, // C 窗共享资源数（FIFO 管理，资源号 0..255）
    parameter int SMC_DEPTH = 256 // SMC 模型深度（tag 低 8bit 索引）
) (
    input logic clk,
    input logic rst_n,
    // ---- burst_sch 二级发射（c_task burst，每拍 ≤1） ----
    input logic emit_dma_vld,
    input logic [5:0] emit_dma_tid,
    input logic [3:0] emit_dma_tidx,
    input logic [BURST_W-1:0] emit_dma_burst,
    input logic emit_dma_pre,
    output logic dma_ack,
    // ---- CSR / 线程状态（THM） ----
    input logic [MAX_THREADS*8-1:0] csr_dma_c,
    input logic [MAX_THREADS*384-1:0] csr_cw,
    input logic [MAX_THREADS*TS_ID_W-1:0] thread_curts,
    // ---- CSR.cw 条目更新（→ THM） ----
    output logic cw_upd_vld,
    output logic [5:0] cw_upd_tid,
    output logic [2:0] cw_upd_ind,
    output logic [47:0] cw_upd_data,
    // ---- 资源池状态（burst_sch 发射条件：共享资源空闲数 / 每线程共享占用数） ----
    output logic [9:0] cw_fifo_cnt,
    output logic [MAX_THREADS*3-1:0] th_res_n,
    // ---- 完成（THM cur_ts 推进） ----
    output logic dma_done_vld,
    output logic [5:0] dma_done_tid,
    output logic [3:0] dma_done_tidx,
    // ---- pre_read 预读入口（burst_sch 最高优先级发射；占位：接收即吸收，不执行/不占资源/不回 done） ----
    input logic [3:0] pre_op_vld,
    input logic [23:0] pre_op_tid,
    input logic [79:0] pre_op_addr,
    input logic [3:0] pre_op_type,
    output logic pre_op_ack
);

    import poe_types_pkg::*;

    typedef enum logic [3:0] {
        S_IDLE, S_LOAD,
        S_MISS,
        S_RBA_RD, S_RBA_RD_DONE,
        S_FREE_RBA, S_FREE_REL,
        S_NEXT, S_DONE
    } state_t;

    // ---- C 窗：独享（每线程固定 4 位置）+ 共享（FIFO 管理） ----
    localparam int C_WND_EXCL_PER_TH = 4; // 每线程独享位置数（cw 前 4 项固定映射）
    localparam int C_WND_EXCL_NUM = MAX_THREADS * C_WND_EXCL_PER_TH; // 64×4 = 256
    c_wnd_entry_t c_wnd_excl [MAX_THREADS][C_WND_EXCL_PER_TH];
    c_wnd_entry_t c_wnd_shr [C_WND_SHR_NUM];
    logic [7:0] f_mem [C_WND_SHR_NUM]; // 共享资源号 FIFO（0..255）
    logic [7:0] f_head, f_tail;
    logic [9:0] f_cnt;
    // ---- 每线程已占用共享资源数（上限 4，对应 cw 后 4 项） ----
    logic [2:0] th_res_n_r [MAX_THREADS];
    // ---- SMC 模型（tag 低 8bit 索引，128bit 行） ----
    logic [127:0] smc_mem [SMC_DEPTH];

    state_t st;
    logic [5:0] cur_tid;
    logic [3:0] cur_tidx;
    logic [BURST_W-1:0] cur_burst;
    logic [2:0] cur_dma_id;
    logic [19:0] cur_tag;
    logic cur_op; // 0=loc 1=free
    logic cur_o; // 当前任务 cw 的 o（该线程是否已占据资源）
    logic cur_excl; // 当前任务资源类型：1=独享（cw 条目索引 <4），0=共享
    logic [2:0] cur_tgt_ind; // free 释放目标 cw 条目索引（按 tag 匹配）
    logic [7:0] cur_occ;
    logic [7:0] cur_cn; // 当前 C 窗资源号
    // task 解析结果（S_LOAD 锁存）
    logic t0_ok, t1_ok;
    logic [2:0] t1_dma_id;
    logic [19:0] t1_tag;
    logic t1_op;
    logic t1_excl;
    logic [7:0] t1_occ;
    logic [7:0] t1_cn;
    logic task_is0; // 当前处理 task0（否则 task1）
    logic [127:0] rba_rd_data;
    // cw 更新请求（寄存，下一拍 THM 采样）
    logic cw_upd_vld_r;
    logic [5:0] cw_upd_tid_r;
    logic [2:0] cw_upd_ind_r;
    logic [47:0] cw_upd_data_r;
    // dma_done（寄存，S_DONE 拍置位）
    logic dma_done_vld_r;
    logic [5:0] dma_done_tid_r;
    logic [3:0] dma_done_tidx_r;
    // temp diag：资源收支计数（共享资源）
    logic [15:0] dbg_alloc, dbg_free;

    assign dma_ack = (st == S_IDLE);
    assign cw_fifo_cnt = f_cnt;
    assign cw_upd_vld = cw_upd_vld_r;
    assign cw_upd_tid = cw_upd_tid_r;
    assign cw_upd_ind = cw_upd_ind_r;
    assign cw_upd_data = cw_upd_data_r;
    assign dma_done_vld = dma_done_vld_r;
    assign dma_done_tid = dma_done_tid_r;
    assign dma_done_tidx = dma_done_tidx_r;
    assign pre_op_ack = 1'b1; // 预读入口每拍可收（吸收占位，预读语义待细化）

    always_comb begin
        for (int i = 0; i < MAX_THREADS; i++)
            th_res_n[i*3 +: 3] = th_res_n_r[i];
    end

    // 当前任务对应的 CSR.cw 条目（组合）
    function automatic logic [47:0] cw_entry_of(logic [5:0] tid, logic [2:0] dma_id);
        cw_entry_of = csr_cw[tid*384 +: 384][dma_id*48 +: 48];
    endfunction

    // ---- 当拍资源池事件（合并 FIFO 指针/计数更新，避免同拍多事件覆盖） ----
    logic alloc_ev; // 共享资源申请 1 个（loc）
    logic free_ev; // 共享资源释放 1 个（free）
    always_comb begin
        // loc：独享固定位置不占 FIFO，仅共享申请
        alloc_ev = (st == S_MISS) && !cur_excl;
        // free：独享固定位置不入 FIFO，仅共享释放
        free_ev = (st == S_FREE_REL) && cur_o && !cur_excl;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < MAX_THREADS; i++)
                for (int k = 0; k < C_WND_EXCL_PER_TH; k++) c_wnd_excl[i][k] <= '0;
            for (int i = 0; i < C_WND_SHR_NUM; i++) begin
                c_wnd_shr[i] <= '0;
                f_mem[i] <= i[7:0];
            end
            f_head <= '0;
            f_tail <= '0;
            f_cnt <= C_WND_SHR_NUM[9:0];
            for (int i = 0; i < MAX_THREADS; i++) th_res_n_r[i] <= 3'd0;
            for (int i = 0; i < SMC_DEPTH; i++) smc_mem[i] <= '0;
            st <= S_IDLE;
            cur_tid <= '0;
            cur_tidx <= '0;
            cur_burst <= '0;
            cur_dma_id <= '0;
            cur_tag <= '0;
            cur_op <= 1'b0;
            cur_o <= 1'b0;
            cur_excl <= 1'b0;
            cur_tgt_ind <= '0;
            cur_occ <= '0;
            cur_cn <= '0;
            t0_ok <= 1'b0;
            t1_ok <= 1'b0;
            t1_dma_id <= '0;
            t1_tag <= '0;
            t1_op <= 1'b0;
            t1_excl <= 1'b0;
            t1_occ <= '0;
            t1_cn <= '0;
            task_is0 <= 1'b1;
            rba_rd_data <= '0;
            cw_upd_vld_r <= 1'b0;
            cw_upd_tid_r <= '0;
            cw_upd_ind_r <= '0;
            cw_upd_data_r <= '0;
            dma_done_vld_r <= 1'b0;
            dma_done_tid_r <= '0;
            dma_done_tidx_r <= '0;
            dbg_alloc <= '0;
            dbg_free <= '0;
        end else begin
            cw_upd_vld_r <= 1'b0; // 单拍请求，默认清
            dma_done_vld_r <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (emit_dma_vld && !emit_dma_pre) begin
                        cur_tid <= emit_dma_tid;
                        cur_tidx <= emit_dma_tidx;
                        cur_burst <= emit_dma_burst;
                        st <= S_LOAD;
                    end
                end
                S_LOAD: begin
                    automatic burst_c_t b;
                    automatic logic [7:0] dc;
                    automatic logic [47:0] e0, e1;
                    automatic logic t0c, t1c;
                    b = cur_burst;
                    dc = csr_dma_c[cur_tid*8 +: 8];
                    e0 = cw_entry_of(cur_tid, b.dma_id0);
                    e1 = cw_entry_of(cur_tid, b.dma_id1);
                    t0c = b.c0 && dc[b.dma_id0];
                    t1c = b.vld_cu && b.c1 && dc[b.dma_id1];
                    // 同拍 free 去重：task0/task1 均 free 且 tag 相同 → 只执行 task0
                    if (t0c && t1c && e0[27] && e1[27] && (e0[47:28] == e1[47:28]))
                        t1c = 1'b0;
                    t0_ok <= t0c;
                    t1_ok <= t1c;
                    t1_dma_id <= b.dma_id1;
                    t1_tag <= e1[47:28];
                    t1_op <= e1[27];
                    t1_excl <= (b.dma_id1 < 4); // 占位，free 时按匹配条目覆盖
                    t1_occ <= b.occ_ts1;
                    t1_cn <= 8'd0; // 实际值在 t1c 分支按 tag 查找后锁存
                    task_is0 <= 1'b1;
                    if (t0c) begin
                        automatic int tgt = b.dma_id0;
                        automatic logic [47:0] te = e0;
                        automatic logic texcl = (b.dma_id0 < 4);
                        if (e0[27]) begin // free：释放目标按 tag 在 cw 中匹配（loc 条目）
                            // 互斥锁保证同地址无并发，同线程内同 tag 只有一个 loc 条目，
                            // 直接找 tag 匹配条目（锁保证 loc 已先执行，o=1）
                            tgt = -1;
                            for (int k = 0; k < 8; k++)
                                if (cw_entry_of(cur_tid, k[2:0])[47:28] == e0[47:28]) begin
                                    tgt = k;
                                    break;
                                end
                            te = cw_entry_of(cur_tid, tgt[2:0]);
                            texcl = (tgt < 4); // 资源类型跟随被释放的 loc 条目半区
                        end
                        cur_dma_id <= b.dma_id0;
                        cur_tag <= e0[47:28];
                        cur_op <= e0[27];
                        cur_tgt_ind <= tgt[2:0];
                        cur_o <= te[25];
                        cur_excl <= texcl;
                        cur_occ <= b.occ_ts0;
                        cur_cn <= te[24:17]; // loc 条目的 c_line_num
                        st <= e0[27] ? S_FREE_RBA : S_MISS;
                    end else if (t1c) begin
                        automatic int tgt = b.dma_id1;
                        automatic logic [47:0] te = e1;
                        automatic logic texcl = (b.dma_id1 < 4);
                        if (e1[27]) begin
                            tgt = -1;
                            for (int k = 0; k < 8; k++)
                                if (cw_entry_of(cur_tid, k[2:0])[47:28] == e1[47:28]) begin
                                    tgt = k;
                                    break;
                                end
                            te = cw_entry_of(cur_tid, tgt[2:0]);
                            texcl = (tgt < 4);
                        end
                        cur_dma_id <= b.dma_id1;
                        cur_tag <= e1[47:28];
                        cur_op <= e1[27];
                        cur_tgt_ind <= tgt[2:0];
                        cur_o <= te[25];
                        cur_excl <= texcl;
                        cur_occ <= b.occ_ts1;
                        cur_cn <= te[24:17];
                        t1_cn <= te[24:17];
                        t1_excl <= texcl;
                        st <= e1[27] ? S_FREE_RBA : S_MISS;
                    end else begin
                        st <= S_DONE; // 无有效任务也回 done（THM cur_ts 依赖）
                    end
                end
                S_MISS: begin
                    // loc：按条目类型申请资源（独享固定位置 / 共享 FIFO 出队）
                    // + 写 C 窗 + 更新 cw；RBA 读下一拍发起
                    // （互斥锁保证无冲突，直接申请，不做查重/预存/转交）
                    automatic c_wnd_entry_t we;
                    automatic cw_entry_t ce;
                    automatic logic [7:0] cn;
                    cn = cur_excl ? (cur_tid*4 + cur_dma_id[1:0]) : f_mem[f_head];
                    cur_cn <= cn;
                    we.tag = cur_tag;
                    we.c_line = '0;
                    we.d = 1'b0;
                    we.o = 1'b1;
                    we.r = 1'b0;
                    we.cnt = 9'd0;
                    we.ind = cn;
                    if (cur_excl) c_wnd_excl[cur_tid][cur_dma_id[1:0]] <= we;
                    else c_wnd_shr[cn] <= we;
                    ce.tag = cur_tag;
                    ce.op_type = cur_op;
                    ce.r = 1'b0;
                    ce.o = 1'b1;
                    ce.c_line_num = cn;
                    ce.start_ts = thread_curts[cur_tid*TS_ID_W +: TS_ID_W];
                    ce.occ_ts = cur_occ;
                    ce.rsv = 1'b0;
                    cw_upd_vld_r <= 1'b1;
                    cw_upd_tid_r <= cur_tid;
                    cw_upd_ind_r <= cur_dma_id;
                    cw_upd_data_r <= ce;
                    if (!cur_excl)
                        th_res_n_r[cur_tid] <= th_res_n_r[cur_tid] + 1'b1; // 仅共享占用计数
                    st <= S_RBA_RD;
                end
                S_RBA_RD: begin
                    rba_rd_data <= smc_mem[cur_tag[7:0]];
                    st <= S_RBA_RD_DONE;
                end
                S_RBA_RD_DONE: begin
                    // 数据回填 C 窗 c_line + r=1；更新 cw.r=1
                    if (cur_excl) begin
                        c_wnd_excl[cur_tid][cur_dma_id[1:0]].c_line <= rba_rd_data;
                        c_wnd_excl[cur_tid][cur_dma_id[1:0]].r <= 1'b1;
                    end else begin
                        c_wnd_shr[cur_cn].c_line <= rba_rd_data;
                        c_wnd_shr[cur_cn].r <= 1'b1;
                    end
                    cw_upd_vld_r <= 1'b1;
                    cw_upd_tid_r <= cur_tid;
                    cw_upd_ind_r <= cur_dma_id;
                    cw_upd_data_r <= {cw_entry_of(cur_tid, cur_dma_id)[47:27],
                                      1'b1, // r=1
                                      cw_entry_of(cur_tid, cur_dma_id)[25:0]};
                    st <= S_NEXT;
                end
                S_FREE_RBA: begin
                    // free：RBA 写（c_line → SMC[tag]），1 拍完成
                    if (cur_o) begin
                        if (cur_excl)
                            smc_mem[cur_tag[7:0]] <= c_wnd_excl[cur_cn >> 2][cur_cn & 3].c_line;
                        else
                            smc_mem[cur_tag[7:0]] <= c_wnd_shr[cur_cn].c_line;
                    end
                    st <= S_FREE_REL;
                end
                S_FREE_REL: begin
                    // 释放（仅该线程已占据资源时执行）：独享固定位置清 o（不入 FIFO），
                    // 共享资源由 free_ev 统一入 FIFO + 占用-1
                    automatic logic [47:0] ce;
                    ce = cw_entry_of(cur_tid, cur_tgt_ind);
                    if (cur_o) begin
                        if (cur_excl) begin
                            c_wnd_excl[cur_cn >> 2][cur_cn & 3].o <= 1'b0;
                        end else begin
                            c_wnd_shr[cur_cn].o <= 1'b0;
                            th_res_n_r[cur_tid] <= th_res_n_r[cur_tid] - 1'b1; // 共享占用-1
                        end
                        ce[25] = 1'b0; // free 线程 o=0
                        cw_upd_vld_r <= 1'b1;
                        cw_upd_tid_r <= cur_tid;
                        cw_upd_ind_r <= cur_tgt_ind;
                        cw_upd_data_r <= ce;
                        st <= S_NEXT;
                    end else begin
                        st <= S_NEXT;
                    end
                end
                S_NEXT: begin
                    if (task_is0 && t1_ok) begin
                        task_is0 <= 1'b0;
                        cur_dma_id <= t1_dma_id;
                        cur_tag <= t1_tag;
                        cur_op <= t1_op;
                        cur_excl <= t1_excl;
                        cur_occ <= t1_occ;
                        cur_cn <= t1_cn;
                        st <= t1_op ? S_FREE_RBA : S_MISS;
                    end else begin
                        st <= S_DONE;
                    end
                end
                S_DONE: begin
                    dma_done_vld_r <= 1'b1;
                    dma_done_tid_r <= cur_tid;
                    dma_done_tidx_r <= cur_tidx;
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
            // ---- 资源池 FIFO 统一更新（合并 alloc/free，避免同拍覆盖） ----
            if (alloc_ev) dbg_alloc <= dbg_alloc + 1'b1;
            if (free_ev) dbg_free <= dbg_free + 1'b1;
            f_head <= f_head + alloc_ev;
            f_tail <= f_tail + free_ev;
            f_cnt <= f_cnt - alloc_ev + free_ev;
        end
    end
endmodule
