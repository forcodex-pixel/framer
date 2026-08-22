// ============================================================================
// POE dma_ctrl 行为模型：c_task（loc= RBA 读 / free= RBA 写）+ SMC/RBA 模型
// 结构（对应《POE系统设计.md》）：
// - 接收 burst_sch 二级发射的 c_task burst（每拍 ≤1，含 ≤2 个 c_task）；
//   按 c0/c1 + CSR.dma_c 判有效任务，按 dma_id 查 CSR.cw 得 op_type（loc/free）
//   与 smc 地址（tag）；**不再做任何 tag 相同/冲突检查**（同地址互斥由 THM
//   锁在一级发射阻塞，同地址操作在线程描述中约束为携带相同锁 id）。
// - C 窗每线程独享 8 个固定位置（cw 8 项全映射），**无 loc/free 生命周期**：
//   c_wnd 仅作纯数据存储（c_line），不维护 o/d/r/cnt/ind 占用状态；
//   CSR.cw 字段（r/o/c_line_num/start_ts/occ_ts/rsv）保留定义，dma_ctrl 不回写。
// - loc（0）：RBA 读 SMC[tag] → 回填 c_wnd[tid][dma_id].c_line（2 拍延迟）；
// - free（1）：RBA 写 c_wnd[tid][dma_id].c_line → SMC[tag]（1 拍延迟）；
// - dma_done 表示 burst 执行完成（THM cur_ts 推进用）。
// - pre 插队 burst：不执行、不回 done（占位，预读语义待细化）。
// ============================================================================
module poe_dma_ctrl #(
    parameter int MAX_THREADS = 64,
    parameter int TS_ID_W = 6,
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
    // ---- CSR（THM，只读：dma_c 任务掩码 / cw 操作表） ----
    input logic [MAX_THREADS*8-1:0] csr_dma_c,
    input logic [MAX_THREADS*384-1:0] csr_cw,
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
        S_RBA_RD, S_RBA_RD_DONE,
        S_FREE_RBA,
        S_NEXT, S_DONE
    } state_t;

    // ---- C 窗：每线程独享 8 个固定位置（纯数据存储，无占用管理） ----
    localparam int C_WND_PER_TH = 8; // 每线程独享位置数（= cw 项数）
    c_wnd_entry_t c_wnd [MAX_THREADS][C_WND_PER_TH];
    // ---- SMC 模型（tag 低 8bit 索引，128bit 行） ----
    logic [127:0] smc_mem [SMC_DEPTH];

    state_t st;
    logic [5:0] cur_tid;
    logic [3:0] cur_tidx;
    logic [BURST_W-1:0] cur_burst;
    logic [2:0] cur_dma_id;
    logic [19:0] cur_tag;
    logic cur_op; // 0=loc 1=free
    // task 解析结果（S_LOAD 锁存）
    logic t0_ok, t1_ok;
    logic [2:0] t1_dma_id;
    logic [19:0] t1_tag;
    logic t1_op;
    logic task_is0; // 当前处理 task0（否则 task1）
    logic [127:0] rba_rd_data;
    // dma_done（寄存，S_DONE 拍置位）
    logic dma_done_vld_r;
    logic [5:0] dma_done_tid_r;
    logic [3:0] dma_done_tidx_r;
    assign dma_ack = (st == S_IDLE);
    assign dma_done_vld = dma_done_vld_r;
    assign dma_done_tid = dma_done_tid_r;
    assign dma_done_tidx = dma_done_tidx_r;
    assign pre_op_ack = 1'b1; // 预读入口每拍可收（吸收占位，预读语义待细化）

    // 当前任务对应的 CSR.cw 条目（组合）
    function automatic logic [47:0] cw_entry_of(logic [5:0] tid, logic [2:0] dma_id);
        cw_entry_of = csr_cw[tid*384 +: 384][dma_id*48 +: 48];
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < MAX_THREADS; i++)
                for (int k = 0; k < C_WND_PER_TH; k++) c_wnd[i][k] <= '0;
            for (int i = 0; i < SMC_DEPTH; i++) smc_mem[i] <= '0;
            st <= S_IDLE;
            cur_tid <= '0;
            cur_tidx <= '0;
            cur_burst <= '0;
            cur_dma_id <= '0;
            cur_tag <= '0;
            cur_op <= 1'b0;
            t0_ok <= 1'b0;
            t1_ok <= 1'b0;
            t1_dma_id <= '0;
            t1_tag <= '0;
            t1_op <= 1'b0;
            task_is0 <= 1'b1;
            rba_rd_data <= '0;
            dma_done_vld_r <= 1'b0;
            dma_done_tid_r <= '0;
            dma_done_tidx_r <= '0;
        end else begin
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
                    t0_ok <= t0c;
                    t1_ok <= t1c;
                    t1_dma_id <= b.dma_id1;
                    t1_tag <= e1[47:28];
                    t1_op <= e1[27];
                    task_is0 <= 1'b1;
                    if (t0c) begin
                        cur_dma_id <= b.dma_id0;
                        cur_tag <= e0[47:28];
                        cur_op <= e0[27];
                        st <= e0[27] ? S_FREE_RBA : S_RBA_RD;
                    end else if (t1c) begin
                        cur_dma_id <= b.dma_id1;
                        cur_tag <= e1[47:28];
                        cur_op <= e1[27];
                        st <= e1[27] ? S_FREE_RBA : S_RBA_RD;
                    end else begin
                        st <= S_DONE; // 无有效任务也回 done（THM cur_ts 依赖）
                    end
                end
                S_RBA_RD: begin
                    rba_rd_data <= smc_mem[cur_tag[7:0]];
                    st <= S_RBA_RD_DONE;
                end
                S_RBA_RD_DONE: begin
                    // loc：RBA 读回填 C 窗 c_line（纯数据存储，无占用状态管理/不回写 cw）
                    c_wnd[cur_tid][cur_dma_id].c_line <= rba_rd_data;
                    st <= S_NEXT;
                end
                S_FREE_RBA: begin
                    // free：RBA 写（C 窗 c_line → SMC[tag]），1 拍完成
                    smc_mem[cur_tag[7:0]] <= c_wnd[cur_tid][cur_dma_id].c_line;
                    st <= S_NEXT;
                end
                S_NEXT: begin
                    if (task_is0 && t1_ok) begin
                        task_is0 <= 1'b0;
                        cur_dma_id <= t1_dma_id;
                        cur_tag <= t1_tag;
                        cur_op <= t1_op;
                        st <= t1_op ? S_FREE_RBA : S_RBA_RD;
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
        end
    end
endmodule
