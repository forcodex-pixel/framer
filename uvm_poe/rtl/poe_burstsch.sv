// POE burst_sch（二级发射调度器）：
// - 从 th_sch 的 2 个 burst 队列（q0/q1）各自独立判断队头可发射，每拍 ≤2 个
// - 可发射条件（所有 burst 均须满足）：
//   ① burst.ts == 线程当前 cur_ts（cur_ts 由 cu_done/dma_done 推进，队列中不会出现
//      当前 ts 以前的 burst）；pre_read 插队 burst 无线程归属，跳过本检查
//   ② 不满足"O 窗反压 且 burst 涉及 O 窗操作"（owin_bp=1 时 tr=1 的 burst 阻塞）
// - c_task（burst_type=1）新增条件（不替换公共条件）：按 c0/c1 判任务是否需发 dma_ctrl，
//   需要时查 CSR.dma_c[dma_id] 确认生效（bit=1）。C 窗每线程独享 8 个固定位置
//   （无共享池/占用计数），c_task 不再需要资源预检，无条件放行
//   （lock/free 由 ts 级互斥锁保证成对与互斥）。
//   pre_read 插队 burst 暂不占资源（跳过）。
// - 发射路由：burst_type=0（i/v_task）→ CU0/CU1 两个单元（q0→CU0、q1→CU1）；
//   =1（c_task）→ dma_ctrl（dma 单路，两路 c_task 同拍只发 q0，q1 保留下一拍）
//   （操作指令格式见方案文档：vld+th_id+op_type+smc_addr，每拍 ≤4 路）
// - owin_bp 由外部提供（O 窗资源池内部设计后续补充）
module poe_burstsch #(
    parameter int MAX_THREADS = 64,
    parameter int MAX_TS = 16,
    parameter int TS_ID_W = 6
    ) (
    input logic clk,
    input logic rst_n,
    // ---- th_sch burst 队列（q0/q1） ----
    input logic q0_vld,
    input logic [5:0] q0_tid,
    input logic [TS_ID_W-1:0] q0_ts,
    input logic [3:0] q0_tidx,
    input logic [BURST_W-1:0] q0_burst,
    input logic q0_pre,
    output logic q0_ack,
    input logic q1_vld,
    input logic [5:0] q1_tid,
    input logic [TS_ID_W-1:0] q1_ts,
    input logic [3:0] q1_tidx,
    input logic [BURST_W-1:0] q1_burst,
    input logic q1_pre,
    output logic q1_ack,
    // ---- 线程状态 ----
    input logic [MAX_THREADS*TS_ID_W-1:0] thread_curts,
    // ---- O 窗反压（资源池设计后续补充） ----
    input logic owin_bp,
    // ---- 发射输出（每拍 ≤2）：i/v_task → CU0/CU1；c_task → dma_ctrl ----
    output logic emit_cu_vld0,
    output logic [5:0] emit_cu_tid0,
    output logic [3:0] emit_cu_tidx0,
    output logic [BURST_W-1:0] emit_cu_burst0,
    input logic cu0_ack,
    output logic emit_cu_vld1,
    output logic [5:0] emit_cu_tid1,
    output logic [3:0] emit_cu_tidx1,
    output logic [BURST_W-1:0] emit_cu_burst1,
    input logic cu1_ack,
    output logic emit_dma_vld,
    output logic [5:0] emit_dma_tid,
    output logic [3:0] emit_dma_tidx,
    output logic [BURST_W-1:0] emit_dma_burst,
    output logic emit_dma_pre, // pre_read 插队 burst（dma_ctrl 不占资源/不回 done）
    input logic dma_ack,
    // ---- pre_read 预读接口（THM → burst_sch：8 深缓存，最高优先级调度） ----
    input logic [3:0] pre_vld,
    input logic [23:0] pre_tid,
    input logic [79:0] pre_dma_addr,
    input logic [3:0] pre_op,
    output logic pre_buf_rdy,
    // ---- 预读发射（→ dma_ctrl 预读入口，每拍 ≤4 组） ----
    output logic [3:0] pre_op_vld,
    output logic [23:0] pre_op_tid,
    output logic [79:0] pre_op_addr,
    output logic [3:0] pre_op_type,
    input logic pre_op_ack
    );

    import poe_types_pkg::*;

    // ---- pre_read 预读缓存（8 深 × {v, tid, dma_addr, op}） ----
    localparam int PRE_BUF_DEPTH = 8;
    logic [27:0] pre_buf [PRE_BUF_DEPTH];
    logic [2:0] pre_head, pre_tail;
    logic [3:0] pre_cnt;
    logic [1:0] pre_out_n; // 本拍预读发射组数（0..4）

    assign pre_buf_rdy = (pre_cnt <= PRE_BUF_DEPTH - 4); // 需容纳每拍最多 4 组
    // 组合：预读缓存队头最多 4 组输出（最高优先级，dma_ctrl 每拍可收 4 组）
    always_comb begin
        pre_out_n = 2'd0;
        for (int k = 0; k < 4; k++) begin
            if (k < pre_cnt) begin
                automatic int idx = (pre_head + k) % PRE_BUF_DEPTH;
                pre_op_vld[k] = pre_buf[idx][27];
                pre_op_tid[k*6 +: 6] = pre_buf[idx][26:21];
                pre_op_addr[k*20 +: 20] = pre_buf[idx][20:1];
                pre_op_type[k] = pre_buf[idx][0];
                pre_out_n = k[1:0] + 1'b1;
            end else begin
                pre_op_vld[k] = 1'b0;
                pre_op_tid[k*6 +: 6] = 6'd0;
                pre_op_addr[k*20 +: 20] = 20'd0;
                pre_op_type[k] = 1'b0;
            end
        end
    end

    // 本拍预读写入组数（0..4）
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
            // 写入（THM 预读转发，受 pre_buf_rdy 门控，≤4 组）
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
            // 出队（dma_ctrl 预读入口接收后推进）
            if (pre_op_ack && (pre_out_n != 2'd0)) begin
                pre_head <= pre_head + pre_out_n;
            end
            // 计数合并：写 + 入队数，读 - 出队数
            pre_cnt <= pre_cnt + pre_wr_n
                       - ((pre_op_ack && (pre_out_n != 2'd0)) ? {2'd0, pre_out_n} : 4'd0);
        end
    end

    logic rr; // 0=q0 优先，1=q1 优先

    // 组合：检查 q0/q1 队头是否可发射（公共条件 + c_task 的资源条件 + 目的端 ack）
    logic emit0_cond, emit1_cond; // 源侧条件
    logic avail0, avail1; // 源侧条件 && 目的端可接收
    burst_c_t bc0, bc1;
    always_comb begin
        bc0 = q0_burst;
        bc1 = q1_burst;
        // 公共条件：pre 插队跳过 ts 检查；tr=1 受 O 窗反压
        emit0_cond = q0_vld &&
    (q0_pre || (q0_ts == thread_curts[q0_tid*TS_ID_W +: TS_ID_W])) &&
                !(owin_bp && bc0.tr);
        emit1_cond = q1_vld &&
    (q1_pre || (q1_ts == thread_curts[q1_tid*TS_ID_W +: TS_ID_W])) &&
                !(owin_bp && bc1.tr);
        // 目的端 ack：i/v → CU0/CU1；c_task → dma_ctrl
        avail0 = emit0_cond && (bc0.burst_type == 1'b0 ? cu0_ack : dma_ack);
        avail1 = emit1_cond && (bc1.burst_type == 1'b0 ? cu1_ack : dma_ack);
    end

    // 双发：q0/q1 各自可发射即发（每拍 ≤2）；dma 单路，两路 c_task 同拍只发 q0
    logic emit0, emit1;
    always_comb begin
        emit0 = avail0;
        emit1 = avail1;
        if (emit0 && emit1 && (bc0.burst_type == 1'b1) && (bc1.burst_type == 1'b1))
            emit1 = 1'b0; // 两路 c_task 同拍只发 q0（dma 单路）
    end

    assign q0_ack = emit0;
    assign q1_ack = emit1;
    assign emit_cu_vld0 = emit0 && (bc0.burst_type == 1'b0);
    assign emit_cu_tid0 = q0_tid;
    assign emit_cu_tidx0 = q0_tidx;
    assign emit_cu_burst0 = q0_burst;
    assign emit_cu_vld1 = emit1 && (bc1.burst_type == 1'b0);
    assign emit_cu_tid1 = q1_tid;
    assign emit_cu_tidx1 = q1_tidx;
    assign emit_cu_burst1 = q1_burst;
    // dma：q0 的 c_task 优先，否则 q1 的
    assign emit_dma_vld = (emit0 && (bc0.burst_type == 1'b1)) ||
                          (emit1 && (bc1.burst_type == 1'b1));
    assign emit_dma_tid = (emit0 && (bc0.burst_type == 1'b1)) ? q0_tid : q1_tid;
    assign emit_dma_tidx = (emit0 && (bc0.burst_type == 1'b1)) ? q0_tidx : q1_tidx;
    assign emit_dma_burst = (emit0 && (bc0.burst_type == 1'b1)) ? q0_burst : q1_burst;
    assign emit_dma_pre = ((emit0 && (bc0.burst_type == 1'b1)) ? q0_pre : q1_pre)
                          && emit_dma_vld;
endmodule
