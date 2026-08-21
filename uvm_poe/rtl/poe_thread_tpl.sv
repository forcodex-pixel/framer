// ============================================================================
// POE 线程模板池（THM 建线程时随机选取，替代激励旁路线程描述）
// 对应《poe_thm实现方案.md》：THM 不再从 KOA/tb 接收完整线程描述，
// 而是在创建线程时从模板池随机选一个模板，解出全部线程字段。
//
// 线程模板字段（与 THM 建线程所需一致）：
//   ts_cnt      ts 数（1..16）
//   ts_bs       每 ts burst 数（16×3bit，ts0 固定 1）
//   ts_id       每 ts 编号（16×6bit，递增可跳转，模拟 ts 跳转）
//   pri         线程 burst 优先级（0 最高）
//   burst_seq   每 ts burst 模式（16 ts × 4 burst × 38bit，含 ts 级锁字段）
//   vtsk_c      CSR vtsk_c：i/v 任务执行掩码
//   dma_c       CSR dma_c：c_task 执行指示（8bit 掩码，对应 cw 8 项）
//   cw          CSR cw：c_task 操作表（8×6B，条目 48bit）
//
// 模板覆盖场景：短 i/v、混合 i/v+c_task、长线程+branch 跳转、DMA 密集、
// 单段持锁、双段持锁（ts 级互斥锁：lock_req/unlock_req 仅 ts 首个 burst 生效）。
// ============================================================================
package poe_thread_tpl_pkg;

    import poe_types_pkg::*;

    localparam int TPL_MAX_TS = 16;
    localparam int TPL_MAX_BURST = 4;

    // ---- 线程模板（打包结构，位宽 = 2984bit） ----
    typedef struct packed {
        logic [4:0] ts_cnt; // ts 数（1..16）
        logic [2:0] pri; // 优先级
        logic [TPL_MAX_TS*3-1:0] ts_bs; // 每 ts burst 数
        logic [TPL_MAX_TS*6-1:0] ts_id; // 每 ts 编号
        logic [TPL_MAX_TS*TPL_MAX_BURST*BURST_W-1:0] burst_seq; // 每 ts burst 模式
        logic [7:0] vtsk_c;
        logic [7:0] dma_c;
        logic [383:0] cw; // 8×6B
    } thread_tpl_t;

    localparam int N_TPL = 6; // 模板数量

    // ==================== 辅助构造函数 ====================

    // i/v_task burst（38bit；锁字段可单独设置）
    function automatic logic [BURST_W-1:0] tpl_iv(
        input logic st, input logic [2:0] ts_len,
        input logic branch, input logic tr, input logic [2:0] tsk0);
        burst_iv_t b;
        b = '0;
        b.lock_id = 4'd0;
        b.unlock_req = 1'b0;
        b.lock_req = 1'b0;
        b.st = st;
        b.tr = tr;
        b.ts_len = ts_len;
        b.branch = branch;
        b.burst_type = 1'b0; // i/v
        b.vld_cu = 1'b0; // 单任务
        b.tsk_id0 = tsk0;
        b.c0 = 1'b1;
        b.tsk_id1 = 3'd0;
        b.c1 = 1'b0;
        b.sub_pc0 = 8'h10;
        b.sub_pc1 = 8'h00;
        tpl_iv = b;
    endfunction

    // c_task burst（38bit，单任务；dma_id 与 cw 配对）
    function automatic logic [BURST_W-1:0] tpl_c(
        input logic st, input logic [2:0] dma0, input logic [7:0] occ0);
        burst_c_t b;
        b = '0;
        b.lock_id = 4'd0;
        b.unlock_req = 1'b0;
        b.lock_req = 1'b0;
        b.st = st;
        b.tr = 1'b0;
        b.rev = 4'd0;
        b.burst_type = 1'b1; // c_task
        b.vld_cu = 1'b0; // 单任务
        b.dma_id0 = dma0;
        b.c0 = 1'b1;
        b.dma_id1 = 3'd0;
        b.c1 = 1'b0;
        b.occ_ts0 = occ0;
        b.occ_ts1 = 8'd1;
        tpl_c = b;
    endfunction

    // 对 burst 设置 ts 级锁字段（lock/unlock 仅 st=1 有效）
    function automatic logic [BURST_W-1:0] tpl_set_lock(
        input logic [BURST_W-1:0] b,
        input logic [3:0] lock_id, input logic lock_req, input logic unlock_req);
        automatic logic [BURST_W-1:0] r = b;
        r[BURST_W-1 -: 4] = lock_id;
        r[BURST_W-5] = unlock_req;
        r[BURST_W-6] = lock_req;
        tpl_set_lock = r;
    endfunction

    // cw 条目（48bit）
    function automatic logic [47:0] tpl_cw_entry(
        input logic [19:0] tag, input logic op_type);
        cw_entry_t ce;
        ce = '0;
        ce.tag = tag;
        ce.op_type = op_type; // 0=loc 1=free
        ce.r = 1'b0;
        ce.o = 1'b0;
        ce.c_line_num = 8'd0;
        ce.start_ts = 8'd0;
        ce.occ_ts = 8'd4;
        ce.rsv = 1'b0;
        tpl_cw_entry = ce;
    endfunction

    // 把 4 个 burst 写入模板第 k 个 ts（每 ts 152bit）
    function automatic void tpl_put_ts(inout thread_tpl_t t, input int k,
        input logic [BURST_W-1:0] b0, input logic [BURST_W-1:0] b1,
        input logic [BURST_W-1:0] b2, input logic [BURST_W-1:0] b3);
        t.burst_seq[k*(4*BURST_W) +: 4*BURST_W] = {b3, b2, b1, b0};
    endfunction

    // ==================== 模板定义 ====================

    // ---- T0：短 i/v 线程（2 ts，纯计算，无 c_task / 无锁） ----
    function automatic thread_tpl_t tpl_short_iv();
        thread_tpl_t t;
        t = '0;
        t.ts_cnt = 5'd2;
        t.pri = 3'd1;
        t.ts_bs[2:0] = 3'd1; // ts0 固定 1
        t.ts_bs[5:3] = 3'd2; // ts1 2 条
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        tpl_put_ts(t, 0, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), '0, '0, '0);
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd1),
                        tpl_iv(1'b0, 3'd2, 1'b1, 1'b0, 3'd2), '0, '0);
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h00;
        t.cw = '0;
        tpl_short_iv = t;
    endfunction

    // ---- T1：混合线程（4 ts，1 对共享 c_task loc/free，无锁） ----
    function automatic thread_tpl_t tpl_mix_1pair();
        thread_tpl_t t;
        t = '0;
        t.ts_cnt = 5'd4;
        t.pri = 3'd2;
        t.ts_bs[2:0] = 3'd1;
        t.ts_bs[5:3] = 3'd2;
        t.ts_bs[8:6] = 3'd2;
        t.ts_bs[11:9] = 3'd1;
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        t.ts_id[17:12] = 6'd2;
        t.ts_id[23:18] = 6'd3;
        tpl_put_ts(t, 0, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), '0, '0, '0);
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b0, 1'b1, 3'd1),
                        tpl_c(1'b0, 3'd4, 8'd4), '0, '0); // ts1: dma4 loc（共享）
        tpl_put_ts(t, 2, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd2),
                        tpl_c(1'b0, 3'd5, 8'd4), '0, '0); // ts2: dma5 free（共享）
        tpl_put_ts(t, 3, tpl_iv(1'b1, 3'd1, 1'b1, 1'b0, 3'd3), '0, '0, '0);
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h30; // dma4/5 有效
        t.cw = {6{48'd0}};
        t.cw[239:192] = tpl_cw_entry(20'h1_0001, 1'b0); // cw[4] loc tag
        t.cw[287:240] = tpl_cw_entry(20'h1_0001, 1'b1); // cw[5] free tag
        tpl_mix_1pair = t;
    endfunction

    // ---- T2：长线程 + branch 跳转（8 ts，1 对共享 c_task，ts 编号跳转） ----
    function automatic thread_tpl_t tpl_long_branch();
        thread_tpl_t t;
        t = '0;
        t.ts_cnt = 5'd8;
        t.pri = 3'd3;
        // 每 ts burst 数
        t.ts_bs[2:0] = 3'd1;
        t.ts_bs[5:3] = 3'd2;
        t.ts_bs[8:6] = 3'd1;
        t.ts_bs[11:9] = 3'd2;
        t.ts_bs[14:12] = 3'd1;
        t.ts_bs[17:15] = 3'd2;
        t.ts_bs[20:18] = 3'd2;
        t.ts_bs[23:21] = 3'd1;
        // ts 编号递增可跳转（模拟跳转目标）
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        t.ts_id[17:12] = 6'd3;
        t.ts_id[23:18] = 6'd4;
        t.ts_id[29:24] = 6'd7;
        t.ts_id[35:30] = 6'd9;
        t.ts_id[41:36] = 6'd11;
        t.ts_id[47:42] = 6'd13;
        tpl_put_ts(t, 0, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), '0, '0, '0);
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b1, 1'b0, 3'd1), // branch 在 ts1
                        tpl_iv(1'b0, 3'd2, 1'b0, 1'b1, 3'd2), '0, '0);
        tpl_put_ts(t, 2, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd3), '0, '0, '0);
        tpl_put_ts(t, 3, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd4),
                        tpl_c(1'b0, 3'd4, 8'd4), '0, '0); // loc（共享）
        tpl_put_ts(t, 4, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd5), '0, '0, '0);
        tpl_put_ts(t, 5, tpl_iv(1'b1, 3'd2, 1'b0, 1'b1, 3'd6),
                        tpl_c(1'b0, 3'd5, 8'd4), '0, '0); // free（共享）
        tpl_put_ts(t, 6, tpl_iv(1'b1, 3'd2, 1'b1, 1'b0, 3'd7),
                        tpl_iv(1'b0, 3'd2, 1'b0, 1'b0, 3'd0), '0, '0);
        tpl_put_ts(t, 7, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd1), '0, '0, '0);
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h30;
        t.cw = {6{48'd0}};
        t.cw[239:192] = tpl_cw_entry(20'h2_0002, 1'b0); // cw[4] loc
        t.cw[287:240] = tpl_cw_entry(20'h2_0002, 1'b1); // cw[5] free
        tpl_long_branch = t;
    endfunction

    // ---- T3：DMA 密集线程（5 ts，3 对 c_task：独享 2 对 + 共享 1 对，无锁） ----
    function automatic thread_tpl_t tpl_dma_dense();
        thread_tpl_t t;
        t = '0;
        t.ts_cnt = 5'd5;
        t.pri = 3'd0;
        t.ts_bs[2:0] = 3'd1;
        t.ts_bs[5:3] = 3'd2;
        t.ts_bs[8:6] = 3'd2;
        t.ts_bs[11:9] = 3'd2;
        t.ts_bs[14:12] = 3'd4;
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        t.ts_id[17:12] = 6'd2;
        t.ts_id[23:18] = 6'd3;
        t.ts_id[29:24] = 6'd4;
        tpl_put_ts(t, 0, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), '0, '0, '0);
        // 配对 1（独享 dma0/1）、配对 2（独享 dma2/3）、配对 3（共享 dma4/5）
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd1),
                        tpl_c(1'b0, 3'd0, 8'd4), '0, '0); // loc dma0
        tpl_put_ts(t, 2, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd2),
                        tpl_c(1'b0, 3'd1, 8'd4), '0, '0); // free dma1
        tpl_put_ts(t, 3, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd3),
                        tpl_c(1'b0, 3'd2, 8'd4), '0, '0); // loc dma2
        tpl_put_ts(t, 4, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd4),
                        tpl_c(1'b0, 3'd3, 8'd4), '0, '0); // free dma3
        // 共享配对（dma4/5）在 ts4 尾部
        tpl_put_ts(t, 4, tpl_iv(1'b1, 3'd4, 1'b0, 1'b0, 3'd4),
                        tpl_c(1'b0, 3'd4, 8'd4),
                        tpl_c(1'b0, 3'd5, 8'd4), '0); // loc dma4 + free dma5
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h3F; // dma0..5 有效
        t.cw = '0;
        t.cw[47:0] = tpl_cw_entry(20'h3_0003, 1'b0); // cw[0] loc
        t.cw[95:48] = tpl_cw_entry(20'h3_0003, 1'b1); // cw[1] free
        t.cw[143:96] = tpl_cw_entry(20'h3_0004, 1'b0); // cw[2] loc
        t.cw[191:144] = tpl_cw_entry(20'h3_0004, 1'b1); // cw[3] free
        t.cw[239:192] = tpl_cw_entry(20'h3_0005, 1'b0); // cw[4] loc
        t.cw[287:240] = tpl_cw_entry(20'h3_0005, 1'b1); // cw[5] free
        tpl_dma_dense = t;
    endfunction

    // ---- T4：单段持锁线程（5 ts，ts0 lock、ts3 unlock，锁 ID=0） ----
    function automatic thread_tpl_t tpl_locked();
        thread_tpl_t t;
        automatic logic [BURST_W-1:0] b;
        t = '0;
        t.ts_cnt = 5'd5;
        t.pri = 3'd2;
        t.ts_bs[2:0] = 3'd1;
        t.ts_bs[5:3] = 3'd2;
        t.ts_bs[8:6] = 3'd1;
        t.ts_bs[11:9] = 3'd2;
        t.ts_bs[14:12] = 3'd1;
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        t.ts_id[17:12] = 6'd2;
        t.ts_id[23:18] = 6'd3;
        t.ts_id[29:24] = 6'd4;
        // ts0 首个 burst：加锁 lock_id=0
        b = tpl_set_lock(tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), 4'd0, 1'b1, 1'b0);
        tpl_put_ts(t, 0, b, '0, '0, '0);
        // ts1：i/v + c_task loc（持锁执行）
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd1),
                        tpl_c(1'b0, 3'd4, 8'd4), '0, '0);
        tpl_put_ts(t, 2, tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd2), '0, '0, '0);
        // ts3 首个 burst：解锁 lock_id=0（该 ts 完成时释放）
        b = tpl_set_lock(tpl_iv(1'b1, 3'd2, 1'b0, 1'b1, 3'd3), 4'd0, 1'b0, 1'b1);
        tpl_put_ts(t, 3, b, tpl_c(1'b0, 3'd5, 8'd4), '0, '0);
        tpl_put_ts(t, 4, tpl_iv(1'b1, 3'd1, 1'b1, 1'b0, 3'd4), '0, '0, '0);
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h30;
        t.cw = {6{48'd0}};
        t.cw[239:192] = tpl_cw_entry(20'h4_0004, 1'b0); // cw[4] loc
        t.cw[287:240] = tpl_cw_entry(20'h4_0004, 1'b1); // cw[5] free
        tpl_locked = t;
    endfunction

    // ---- T5：双段持锁线程（6 ts，ts0 lock(1)、ts2 unlock(1)、ts3 lock(2)、ts5 unlock(2)） ----
    function automatic thread_tpl_t tpl_double_lock();
        thread_tpl_t t;
        automatic logic [BURST_W-1:0] b;
        t = '0;
        t.ts_cnt = 5'd6;
        t.pri = 3'd4;
        t.ts_bs[2:0] = 3'd1;
        t.ts_bs[5:3] = 3'd2;
        t.ts_bs[8:6] = 3'd1;
        t.ts_bs[11:9] = 3'd1;
        t.ts_bs[14:12] = 3'd3;
        t.ts_bs[17:15] = 3'd1;
        t.ts_id[5:0] = 6'd0;
        t.ts_id[11:6] = 6'd1;
        t.ts_id[17:12] = 6'd2;
        t.ts_id[23:18] = 6'd4;
        t.ts_id[29:24] = 6'd6;
        t.ts_id[35:30] = 6'd8;
        b = tpl_set_lock(tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd0), 4'd1, 1'b1, 1'b0);
        tpl_put_ts(t, 0, b, '0, '0, '0); // ts0 lock(1)
        tpl_put_ts(t, 1, tpl_iv(1'b1, 3'd2, 1'b0, 1'b0, 3'd1),
                        tpl_iv(1'b0, 3'd2, 1'b0, 1'b1, 3'd2), '0, '0);
        b = tpl_set_lock(tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd3), 4'd1, 1'b0, 1'b1);
        tpl_put_ts(t, 2, b, '0, '0, '0); // ts2 unlock(1)
        b = tpl_set_lock(tpl_iv(1'b1, 3'd1, 1'b0, 1'b0, 3'd4), 4'd2, 1'b1, 1'b0);
        tpl_put_ts(t, 3, b, '0, '0, '0); // ts3 lock(2)
        tpl_put_ts(t, 4, tpl_iv(1'b1, 3'd3, 1'b0, 1'b1, 3'd5),
                        tpl_c(1'b0, 3'd4, 8'd4), // loc dma4（持锁 2）
                        tpl_c(1'b0, 3'd5, 8'd4), '0); // free dma5（配对释放）
        b = tpl_set_lock(tpl_iv(1'b1, 3'd1, 1'b1, 1'b0, 3'd6), 4'd2, 1'b0, 1'b1);
        tpl_put_ts(t, 5, b, '0, '0, '0); // ts5 unlock(2)
        t.vtsk_c = 8'hFF;
        t.dma_c = 8'h30;
        t.cw = {6{48'd0}};
        t.cw[239:192] = tpl_cw_entry(20'h5_0005, 1'b0); // cw[4] loc
        t.cw[287:240] = tpl_cw_entry(20'h5_0005, 1'b1); // cw[5] free
        tpl_double_lock = t;
    endfunction

    // ==================== 池接口 ====================

    // 按索引取模板（越界回退到 T0）
    function automatic thread_tpl_t tpl_get(input int idx);
        case (idx)
            0: tpl_get = tpl_short_iv();
            1: tpl_get = tpl_mix_1pair();
            2: tpl_get = tpl_long_branch();
            3: tpl_get = tpl_dma_dense();
            4: tpl_get = tpl_locked();
            5: tpl_get = tpl_double_lock();
            default: tpl_get = tpl_short_iv();
        endcase
    endfunction

endpackage
