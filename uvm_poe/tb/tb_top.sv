`timescale 1ns/1ps
module tb_top;
    import uvm_pkg::*;
    import ko_pkg::*;
    import poe_types_pkg::*;

    localparam int NUM_OH_PLANES = 4;
    localparam int NUM_X2X_PLANES = 8;

    logic clk = 0;
    logic rst_n = 0;

    always #0.5ns clk = ~clk; // 1 GHz

    koa_if #(.NUM_OH_PLANES(NUM_OH_PLANES), .NUM_X2X_PLANES(NUM_X2X_PLANES))
    u_if(.clk(clk), .rst_n(rst_n));

    koa #(.NUM_OH_PLANES(NUM_OH_PLANES), .NUM_X2X_PLANES(NUM_X2X_PLANES)) dut (
    .clk (clk),
    .rst_n (rst_n),
    .oh_e_vld (u_if.oh_e_vld),
    .oh_e_pri (u_if.oh_e_pri), .oh_e_cid (u_if.oh_e_cid),
    .oh_e_pos (u_if.oh_e_pos), .oh_e_rdy (u_if.oh_e_rdy),
    .oh_i_vld (u_if.oh_i_vld),
    .oh_i_pri (u_if.oh_i_pri), .oh_i_cid (u_if.oh_i_cid),
    .oh_i_pos (u_if.oh_i_pos), .oh_i_rdy (u_if.oh_i_rdy),
    .aps_e_vld(u_if.aps_e_vld),
    .aps_e_pri(u_if.aps_e_pri), .aps_e_cid(u_if.aps_e_cid),
    .aps_e_pos(u_if.aps_e_pos), .aps_e_rdy (u_if.aps_e_rdy),
    .aps_i_vld(u_if.aps_i_vld),
    .aps_i_pri(u_if.aps_i_pri), .aps_i_cid(u_if.aps_i_cid),
    .aps_i_pos(u_if.aps_i_pos), .aps_i_rdy (u_if.aps_i_rdy),
    .alm_vld (u_if.alm_vld),
    .alm_pri (u_if.alm_pri), .alm_cid (u_if.alm_cid),
    .alm_pos (u_if.alm_pos), .alm_rdy (u_if.alm_rdy),
    .u_e_vld (u_if.u_e_vld),
    .u_e_pri (u_if.u_e_pri), .u_e_rdy (u_if.u_e_rdy),
    .u_i_vld (u_if.u_i_vld),
    .u_i_pri (u_if.u_i_pri), .u_i_rdy (u_if.u_i_rdy),
    .out_vld (u_if.out_vld),
    .out_pri (u_if.out_pri),
    .out_src (u_if.out_src),
    .out_stream(u_if.out_stream),
    .out_cid (u_if.out_cid),
    .out_pos (u_if.out_pos),
    .ko_pre_vld(ko_pre_vld), .ko_dma_addr(ko_dma_addr), .ko_pre_op(ko_pre_op),
    .out_pre_vld(u_if.out_pre_vld), .out_dma_addr(u_if.out_dma_addr), .out_pre_op(u_if.out_pre_op)
    );

    // ================= POE 链路：KOA → THM → th_sch → burst_sch → CU/EU + dma_ctrl =================
    // 线程描述旁路（模拟 I_BUF_A 内容）：ts_cnt/bs_cnt/pri + 每 ts 的 burst 模式（38bit×4）
    logic ko_rdy;
    logic [63:0] ready_mask;
    logic [191:0] ready_pri;
    logic [383:0] ready_burst_ts; // 每线程 6bit ts 编号
    logic [255:0] ready_burst_tidx; // 每线程 4bit ts 序号（done 归属）
    logic [383:0] ready_curts; // 每线程 6bit 当前 ts 编号
    logic [2431:0] ready_burst;
    logic iss_vld0, iss_vld1;
    logic [5:0] iss_tid0, iss_tid1;
    // ---- pre_read 预读接口（KOA→THM→burst_sch→dma_ctrl） ----
    logic [3:0] ko_pre_vld; // KOA 入口预读指示（tb_top 激励驱动）
    logic [79:0] ko_dma_addr;
    logic [3:0] ko_pre_op;
    logic [3:0] out_pre_vld; // KOA 出口（随报文对齐 → THM）
    logic [79:0] out_dma_addr;
    logic [3:0] out_pre_op;
    logic [3:0] pre_vld; // THM → burst_sch 预读转发
    logic [23:0] pre_tid;
    logic [79:0] pre_dma_addr;
    logic [3:0] pre_op;
    logic pre_buf_rdy; // burst_sch 预读缓存空间
    // ---- th_sch 槽池视图（q0/q1 i/v + c_task 缓存，逐槽扁平） ----
    logic [7:0] q0_vld, q0_pre, q1_vld, q1_pre;
    logic [23:0] q0_pri, q1_pri;
    logic [47:0] q0_tid, q1_tid;
    logic [47:0] q0_ts, q1_ts;
    logic [31:0] q0_tidx, q1_tidx;
    logic [303:0] q0_burst, q1_burst;
    logic [7:0] q0_ack, q1_ack;
    logic [15:0] ct_vld;
    logic [47:0] ct_pri;
    logic [95:0] ct_tid;
    logic [63:0] ct_tidx;
    logic [95:0] ct_ts;
    logic [47:0] ct_dma;
    logic [31:0] ct_tag;
    logic [15:0] ct_op;
    logic [15:0] ct_ack;
    logic [511:0] csr_dma_c;
    logic [24575:0] csr_cw;
    logic [7:0] pre_dma_c;
    logic [255:0] pre_cw;
    logic owin_bp;
    // ---- EU（2 个，各含 4 个 CU 桩）----
    logic emit_eu_vld0, eu0_ack;
    logic [5:0] emit_eu_tid0;
    logic [3:0] emit_eu_tidx0;
    logic [BURST_W-1:0] emit_eu_burst0;
    logic emit_eu_vld1, eu1_ack;
    logic [5:0] emit_eu_tid1;
    logic [3:0] emit_eu_tidx1;
    logic [BURST_W-1:0] emit_eu_burst1;
    logic eu_done_vld0, eu_done_vld1;
    logic [5:0] eu_done_tid0, eu_done_tid1;
    logic [3:0] eu_done_tidx0, eu_done_tidx1;
    // ---- c_task 发射/done（burst_sch 发射即完成；推入 dma_ctrl FIFO）----
    logic [3:0] emit_dma_vld;
    logic [23:0] emit_dma_tid;
    logic [15:0] emit_dma_tidx;
    logic [11:0] emit_dma_dma_id;
    logic [7:0] emit_dma_tag;
    logic [3:0] emit_dma_op;
    logic [3:0] dma_full;
    logic [3:0] rba_bp;
    logic [27:0] rba_cnt;
    logic [3:0] dma_done_vld;
    logic [23:0] dma_done_tid;
    logic [15:0] dma_done_tidx;

    poe_thm #(.MAX_THREADS(64)) u_thm (
    .clk(clk), .rst_n(rst_n),
    .ko_vld(u_if.out_vld),
    .ko_stream(u_if.out_stream), .ko_cid(u_if.out_cid), .ko_pos(u_if.out_pos),
    .ko_pre_vld(u_if.out_pre_vld), .ko_dma_addr(u_if.out_dma_addr), .ko_pre_op(u_if.out_pre_op),
    .ko_rdy(ko_rdy),
    .ready_mask(ready_mask),
    .ready_pri(ready_pri),
    .ready_burst_ts(ready_burst_ts),
    .ready_burst_tidx(ready_burst_tidx),
    .ready_curts(ready_curts),
    .ready_burst(ready_burst),
    .iss_vld0(iss_vld0), .iss_tid0(iss_tid0),
    .iss_vld1(iss_vld1), .iss_tid1(iss_tid1),
    .eu_done_vld0(eu_done_vld0), .eu_done_tid0(eu_done_tid0),
    .eu_done_tidx0(eu_done_tidx0),
    .eu_done_vld1(eu_done_vld1), .eu_done_tid1(eu_done_tid1),
    .eu_done_tidx1(eu_done_tidx1),
    .dma_done_vld(dma_done_vld), .dma_done_tid(dma_done_tid),
    .dma_done_tidx(dma_done_tidx),
    .emit_vld(1'b0), .emit_tid(6'd0),
    .pre_buf_rdy(pre_buf_rdy),
    .pre_vld(pre_vld), .pre_tid(pre_tid),
    .pre_dma_addr(pre_dma_addr), .pre_op(pre_op),
    .csr_dma_c(csr_dma_c),
    .csr_cw(csr_cw),
    .pre_dma_c(pre_dma_c),
    .pre_cw(pre_cw)
    );

    poe_thsch #(.MAX_THREADS(64)) u_thsch (
    .clk(clk), .rst_n(rst_n),
    .ready_mask(ready_mask),
    .ready_pri(ready_pri),
    .ready_burst_ts(ready_burst_ts),
    .ready_burst_tidx(ready_burst_tidx),
    .ready_burst(ready_burst),
    .iss_vld0(iss_vld0), .iss_tid0(iss_tid0),
    .iss_vld1(iss_vld1), .iss_tid1(iss_tid1),
    .csr_dma_c(csr_dma_c), .csr_cw(csr_cw),
    .q0_vld(q0_vld), .q0_pre(q0_pre), .q0_pri(q0_pri),
    .q0_tid(q0_tid), .q0_ts(q0_ts), .q0_tidx(q0_tidx),
    .q0_burst(q0_burst), .q0_ack(q0_ack),
    .q1_vld(q1_vld), .q1_pre(q1_pre), .q1_pri(q1_pri),
    .q1_tid(q1_tid), .q1_ts(q1_ts), .q1_tidx(q1_tidx),
    .q1_burst(q1_burst), .q1_ack(q1_ack),
    .ct_vld(ct_vld), .ct_pri(ct_pri), .ct_tid(ct_tid),
    .ct_tidx(ct_tidx), .ct_ts(ct_ts), .ct_dma(ct_dma), .ct_tag(ct_tag),
    .ct_op(ct_op), .ct_ack(ct_ack)
    );

    poe_burstsch #(.MAX_THREADS(64)) u_burstsch (
    .clk(clk), .rst_n(rst_n),
    .q0_vld(q0_vld), .q0_pre(q0_pre), .q0_pri(q0_pri),
    .q0_tid(q0_tid), .q0_ts(q0_ts), .q0_tidx(q0_tidx),
    .q0_burst(q0_burst), .q0_ack(q0_ack),
    .q1_vld(q1_vld), .q1_pre(q1_pre), .q1_pri(q1_pri),
    .q1_tid(q1_tid), .q1_ts(q1_ts), .q1_tidx(q1_tidx),
    .q1_burst(q1_burst), .q1_ack(q1_ack),
    .ct_vld(ct_vld), .ct_pri(ct_pri), .ct_tid(ct_tid),
    .ct_tidx(ct_tidx), .ct_ts(ct_ts), .ct_dma(ct_dma), .ct_tag(ct_tag),
    .ct_op(ct_op), .ct_ack(ct_ack),
    .thread_curts(ready_curts),
    .owin_bp(owin_bp),
    .emit_eu_vld0(emit_eu_vld0), .emit_eu_tid0(emit_eu_tid0),
    .emit_eu_tidx0(emit_eu_tidx0), .emit_eu_burst0(emit_eu_burst0),
    .eu0_ack(eu0_ack),
    .emit_eu_vld1(emit_eu_vld1), .emit_eu_tid1(emit_eu_tid1),
    .emit_eu_tidx1(emit_eu_tidx1), .emit_eu_burst1(emit_eu_burst1),
    .eu1_ack(eu1_ack),
    .emit_dma_vld(emit_dma_vld), .emit_dma_tid(emit_dma_tid),
    .emit_dma_tidx(emit_dma_tidx), .emit_dma_dma_id(emit_dma_dma_id),
    .emit_dma_tag(emit_dma_tag), .emit_dma_op(emit_dma_op),
    .dma_full(dma_full),
    .dma_done_vld(dma_done_vld), .dma_done_tid(dma_done_tid),
    .dma_done_tidx(dma_done_tidx),
    .pre_vld(pre_vld), .pre_tid(pre_tid),
    .pre_dma_addr(pre_dma_addr), .pre_op(pre_op),
    .pre_buf_rdy(pre_buf_rdy)
    );

    poe_eu_stub #(.LATENCY(1)) u_eu0 (
    .clk(clk), .rst_n(rst_n),
    .emit_eu_vld(emit_eu_vld0), .emit_eu_tid(emit_eu_tid0),
    .emit_eu_tidx(emit_eu_tidx0), .emit_eu_burst(emit_eu_burst0),
    .eu_ack(eu0_ack),
    .eu_done_vld(eu_done_vld0), .eu_done_tid(eu_done_tid0), .eu_done_tidx(eu_done_tidx0)
    );

    poe_eu_stub #(.LATENCY(1)) u_eu1 (
    .clk(clk), .rst_n(rst_n),
    .emit_eu_vld(emit_eu_vld1), .emit_eu_tid(emit_eu_tid1),
    .emit_eu_tidx(emit_eu_tidx1), .emit_eu_burst(emit_eu_burst1),
    .eu_ack(eu1_ack),
    .eu_done_vld(eu_done_vld1), .eu_done_tid(eu_done_tid1), .eu_done_tidx(eu_done_tidx1)
    );

    poe_dma_ctrl u_dma (
    .clk(clk), .rst_n(rst_n),
    .push_vld(emit_dma_vld),
    .push_tid(emit_dma_tid),
    .push_tidx(emit_dma_tidx),
    .push_dma_id(emit_dma_dma_id),
    .push_tag(emit_dma_tag),
    .push_op(emit_dma_op),
    .fifo_full(dma_full),
    .rba_bp(rba_bp),
    .rba_cnt(rba_cnt)
    );

    // O 窗反压占位：固定 0（O 窗池设计后续补充）
    assign owin_bp = 1'b0;
    // ---- 临时调试日志 ----
    integer thm_logf;
    logic [3:0] rba_bp_prev;
    initial thm_logf = $fopen("thm_dbg.log", "w");
    always @(posedge clk) begin
        burst_c_t cb0, cb1;
        // RBA 反压事件（复位握手后才有意义）
        if (rst_n && (rba_bp !== rba_bp_prev)) begin
            $fdisplay(thm_logf, "RBA_BP t=%0t bp=%b cnt=%0d/%0d/%0d/%0d",
            $time, rba_bp, rba_cnt[6:0], rba_cnt[13:7],
            rba_cnt[20:14], rba_cnt[27:21]);
            rba_bp_prev = rba_bp;
        end
        // 槽池异常检查：vld 且非 pre 且 ts < cur_ts
        for (int i = 0; i < 8; i++) begin
            if (q0_vld[i] && !q0_pre[i] &&
                (q0_ts[i*6 +: 6] < ready_curts[q0_tid[i*6 +: 6]*6 +: 6]))
                $fdisplay(thm_logf, "OLD_BURST0 t=%0t slot=%0d q0ts=%0d curts=%0d tid=%0d",
                $time, i, q0_ts[i*6 +: 6],
                ready_curts[q0_tid[i*6 +: 6]*6 +: 6], q0_tid[i*6 +: 6]);
            if (q1_vld[i] && !q1_pre[i] &&
                (q1_ts[i*6 +: 6] < ready_curts[q1_tid[i*6 +: 6]*6 +: 6]))
                $fdisplay(thm_logf, "OLD_BURST1 t=%0t slot=%0d q1ts=%0d curts=%0d tid=%0d",
                $time, i, q1_ts[i*6 +: 6],
                ready_curts[q1_tid[i*6 +: 6]*6 +: 6], q1_tid[i*6 +: 6]);
        end
        if (emit_eu_vld0 || emit_eu_vld1) begin
            burst_iv_t eb;
            if (emit_eu_vld1) begin
                eb = emit_eu_burst1;
                $fdisplay(thm_logf, "EMIT_EU1 t=%0t tid=%0d ts=%0d st=%0d tr=%0d ts_len=%0d branch=%0d vld=%0d tsk=%0d/%0d c=%0d/%0d pc=%0d need=%0d tscnt=%0d",
                $time, emit_eu_tid1, ready_curts[emit_eu_tid1*6 +: 6],
                eb.st, eb.tr, eb.ts_len, eb.branch, eb.vld_cu,
                eb.tsk_id0, eb.tsk_id1, eb.c0, eb.c1,
                u_thm.th_bs_pc[emit_eu_tid1], u_thm.th_need[emit_eu_tid1][u_thm.th_ts_idx[emit_eu_tid1]],
                u_thm.th_ts_n[emit_eu_tid1]);
            end else begin
                eb = emit_eu_burst0;
                $fdisplay(thm_logf, "EMIT_EU t=%0t tid=%0d ts=%0d st=%0d tr=%0d ts_len=%0d branch=%0d vld=%0d tsk=%0d/%0d c=%0d/%0d pc=%0d need=%0d tscnt=%0d",
                $time, emit_eu_tid0, ready_curts[emit_eu_tid0*6 +: 6],
                eb.st, eb.tr, eb.ts_len, eb.branch, eb.vld_cu,
                eb.tsk_id0, eb.tsk_id1, eb.c0, eb.c1,
                u_thm.th_bs_pc[emit_eu_tid0], u_thm.th_need[emit_eu_tid0][u_thm.th_ts_idx[emit_eu_tid0]],
                u_thm.th_ts_n[emit_eu_tid0]);
            end
        end
        for (int d = 0; d < 4; d++)
            if (emit_dma_vld[d])
                $fdisplay(thm_logf, "EMIT_DMA t=%0t dse=%0d tid=%0d tidx=%0d dma_id=%0d tag=%0d op=%0d",
                $time, d, emit_dma_tid[d*6 +: 6], emit_dma_tidx[d*4 +: 4],
                emit_dma_dma_id[d*3 +: 3], emit_dma_tag[d*2 +: 2], emit_dma_op[d]);
        if (iss_vld0)
            $fdisplay(thm_logf, "ISS0 t=%0t tid=%0d curts=%0d burst_ts=%0d pc=%0d need=%0d tscnt=%0d st=%0d",
        $time, iss_tid0, ready_curts[iss_tid0*6 +: 6],
        ready_burst_ts[iss_tid0*6 +: 6],
        u_thm.th_bs_pc[iss_tid0], u_thm.th_need[iss_tid0][u_thm.th_ts_idx[iss_tid0]],
            u_thm.th_ts_n[iss_tid0], u_thm.th_state[iss_tid0]);
        if (iss_vld1)
            $fdisplay(thm_logf, "ISS1 t=%0t tid=%0d curts=%0d burst_ts=%0d pc=%0d need=%0d tscnt=%0d st=%0d",
        $time, iss_tid1, ready_curts[iss_tid1*6 +: 6],
        ready_burst_ts[iss_tid1*6 +: 6],
        u_thm.th_bs_pc[iss_tid1], u_thm.th_need[iss_tid1][u_thm.th_ts_idx[iss_tid1]],
            u_thm.th_ts_n[iss_tid1], u_thm.th_state[iss_tid1]);
        if (eu_done_vld0)
            $fdisplay(thm_logf, "DONE_EU0 t=%0t tid=%0d curts=%0d csr_curts=%0d done=%0d need=%0d tscnt=%0d pc=%0d st=%0d",
        $time, eu_done_tid0, ready_curts[eu_done_tid0*6 +: 6],
            u_thm.csr[eu_done_tid0].cur_ts,
        u_thm.th_done_acc[eu_done_tid0][u_thm.th_ts_idx[eu_done_tid0]],
        u_thm.th_need[eu_done_tid0][u_thm.th_ts_idx[eu_done_tid0]],
            u_thm.th_ts_n[eu_done_tid0], u_thm.th_bs_pc[eu_done_tid0],
            u_thm.th_state[eu_done_tid0]);
        if (eu_done_vld1)
            $fdisplay(thm_logf, "DONE_EU1 t=%0t tid=%0d curts=%0d csr_curts=%0d done=%0d need=%0d tscnt=%0d pc=%0d st=%0d",
        $time, eu_done_tid1, ready_curts[eu_done_tid1*6 +: 6],
            u_thm.csr[eu_done_tid1].cur_ts,
        u_thm.th_done_acc[eu_done_tid1][u_thm.th_ts_idx[eu_done_tid1]],
        u_thm.th_need[eu_done_tid1][u_thm.th_ts_idx[eu_done_tid1]],
            u_thm.th_ts_n[eu_done_tid1], u_thm.th_bs_pc[eu_done_tid1],
            u_thm.th_state[eu_done_tid1]);
        for (int d = 0; d < 4; d++)
            if (dma_done_vld[d]) begin
                automatic logic [5:0] dt = dma_done_tid[d*6 +: 6];
                $fdisplay(thm_logf, "DONE_DMA t=%0t dse=%0d tid=%0d curts=%0d csr_curts=%0d done=%0d need=%0d tscnt=%0d pc=%0d st=%0d",
                $time, d, dt, ready_curts[dt*6 +: 6],
                    u_thm.csr[dt].cur_ts,
                u_thm.th_done_acc[dt][u_thm.th_ts_idx[dt]],
                u_thm.th_need[dt][u_thm.th_ts_idx[dt]],
                    u_thm.th_ts_n[dt], u_thm.th_bs_pc[dt],
                    u_thm.th_state[dt]);
            end
    end


    // 预读入口激励：每拍随机 0..4 组预读（每组约 1/16 概率，模拟业务流预读请求）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ko_pre_vld <= 4'd0;
            ko_dma_addr <= '0;
            ko_pre_op <= 4'd0;
        end else begin
            ko_pre_vld <= {($urandom % 16) == 0,
                           ($urandom % 16) == 0,
                           ($urandom % 16) == 0,
                           ($urandom % 16) == 0};
            ko_dma_addr <= {$urandom, $urandom, $urandom}; // 80bit（96bit 截断取低）
            ko_pre_op <= {$urandom % 2, $urandom % 2, $urandom % 2, $urandom % 2};
        end
    end

    // 复位握手：复位完成后置位包级 g_reset_done
    initial begin
        rst_n = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        ko_pkg::g_reset_done = 1;
    end

    initial begin
        string wf;
        if ($value$plusargs("WAVE_FILE=%s", wf))
            $dumpfile(wf);
        else
            $dumpfile("wave.vcd");
        $dumpvars(0, tb_top);
    end

    initial begin
        ko_pkg::g_tb_cfg.vif = u_if;
        // 显式引用测试类，防止 Verilator 当作死代码优化掉
        void'(koa_smoke_test::type_id::get());
        run_test();
    end

    final begin
        integer i;
        int idle;
        int n_ready, n_issued, n_done;
        idle = 0;
        n_ready = 0; n_issued = 0; n_done = 0;
        for (i = 0; i < 64; i++) begin
            if (u_thm.th_state[i] == 2'd0) idle++;
            else if (u_thm.th_state[i] == 2'd1) n_ready++;
            else if (u_thm.th_state[i] == 2'd2) n_issued++;
            else n_done++;
        end
        // 校验：所有线程应回到 IDLE（C 窗为纯数据存储，无 loc/free 占用状态）
        $fdisplay(thm_logf, "FINAL idle=%0d ready=%0d issued=%0d done=%0d",
                  idle, n_ready, n_issued, n_done);
        for (i = 0; i < 64; i++)
            if (u_thm.th_state[i] != 2'd0)
                $fdisplay(thm_logf, "NONIDLE tid=%0d st=%0d curts=%0d pc=%0d",
                          i, u_thm.th_state[i], u_thm.th_cur_ts[i], u_thm.th_bs_pc[i]);
        // 槽池残留转储（定位无法发射的 burst）
        for (i = 0; i < 8; i++) begin
            if (q0_vld[i])
                $fdisplay(thm_logf, "POOL0 slot=%0d tid=%0d ts=%0d tidx=%0d curts=%0d pri=%0d",
                          i, q0_tid[i*6 +: 6], q0_ts[i*6 +: 6], q0_tidx[i*4 +: 4],
                          ready_curts[q0_tid[i*6 +: 6]*6 +: 6], q0_pri[i*3 +: 3]);
            if (q1_vld[i])
                $fdisplay(thm_logf, "POOL1 slot=%0d tid=%0d ts=%0d tidx=%0d curts=%0d pri=%0d",
                          i, q1_tid[i*6 +: 6], q1_ts[i*6 +: 6], q1_tidx[i*4 +: 4],
                          ready_curts[q1_tid[i*6 +: 6]*6 +: 6], q1_pri[i*3 +: 3]);
        end
    end
endmodule
