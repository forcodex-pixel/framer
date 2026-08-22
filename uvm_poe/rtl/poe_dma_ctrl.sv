// POE dma_ctrl 行为模型（单 c_task 执行单元，每 DSE 一个实例）：
// - 接收 burst_sch 的 DSE 调度器发射的单个 c_task；
//   解析已在 th_sch 完成（c0/c1 + dma_c 判有效，cw 解析出 tag/op），本单元只执行。
// - loc（op=0）：RBA 读 SMC[tag] → 回填 c_wnd[tid][dma_id].c_line（2 拍）；
// - free（op=1）：RBA 写 c_wnd[tid][dma_id].c_line → SMC[tag]（1 拍）；
// - C 窗每线程独享 8 位置，纯数据存储（只写 c_line），无占用生命周期、不回写 cw；
// - dma_done 表示单个 c_task 执行完成（THM cur_ts 推进用）。
module poe_dma_ctrl #(
    parameter int MAX_THREADS = 64,
    parameter int SMC_DEPTH = 256
) (
    input logic clk,
    input logic rst_n,
    // ---- burst_sch DSE 调度器（单 c_task） ----
    input logic emit_dma_vld,
    input logic [5:0] emit_dma_tid,
    input logic [3:0] emit_dma_tidx,
    input logic [2:0] emit_dma_dma_id,
    input logic [1:0] emit_dma_tag,
    input logic emit_dma_op, // 0=loc(RBA 读) 1=free(RBA 写)
    output logic dma_ack,
    // ---- 完成（THM cur_ts 推进） ----
    output logic dma_done_vld,
    output logic [5:0] dma_done_tid,
    output logic [3:0] dma_done_tidx
);

    import poe_types_pkg::*;

    typedef enum logic [2:0] { S_IDLE, S_RBA_RD, S_RBA_RD_DONE, S_FREE_RBA, S_DONE } state_t;

    // ---- C 窗：每线程独享 8 位置（纯数据存储，只写 c_line） ----
    localparam int C_WND_PER_TH = 8;
    c_wnd_entry_t c_wnd [MAX_THREADS][C_WND_PER_TH];
    // ---- SMC 模型（tag 低 8bit 索引，128bit 行） ----
    logic [127:0] smc_mem [SMC_DEPTH];

    state_t st;
    logic [5:0] cur_tid;
    logic [3:0] cur_tidx;
    logic [2:0] cur_dma_id;
    logic [1:0] cur_tag;
    logic [127:0] rba_rd_data;
    logic done_vld_r;
    logic [5:0] done_tid_r;
    logic [3:0] done_tidx_r;

    assign dma_ack = (st == S_IDLE);
    assign dma_done_vld = done_vld_r;
    assign dma_done_tid = done_tid_r;
    assign dma_done_tidx = done_tidx_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < MAX_THREADS; i++)
                for (int k = 0; k < C_WND_PER_TH; k++) c_wnd[i][k] <= '0;
            for (int i = 0; i < SMC_DEPTH; i++) smc_mem[i] <= '0;
            st <= S_IDLE;
            cur_tid <= '0;
            cur_tidx <= '0;
            cur_dma_id <= '0;
            cur_tag <= '0;
            rba_rd_data <= '0;
            done_vld_r <= 1'b0;
            done_tid_r <= '0;
            done_tidx_r <= '0;
        end else begin
            done_vld_r <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (emit_dma_vld) begin
                        cur_tid <= emit_dma_tid;
                        cur_tidx <= emit_dma_tidx;
                        cur_dma_id <= emit_dma_dma_id;
                        cur_tag <= emit_dma_tag;
                        st <= emit_dma_op ? S_FREE_RBA : S_RBA_RD;
                    end
                end
                S_RBA_RD: begin
                    rba_rd_data <= smc_mem[cur_tag];
                    st <= S_RBA_RD_DONE;
                end
                S_RBA_RD_DONE: begin
                    c_wnd[cur_tid][cur_dma_id].c_line <= rba_rd_data;
                    st <= S_DONE;
                end
                S_FREE_RBA: begin
                    smc_mem[cur_tag] <= c_wnd[cur_tid][cur_dma_id].c_line;
                    st <= S_DONE;
                end
                S_DONE: begin
                    done_vld_r <= 1'b1;
                    done_tid_r <= cur_tid;
                    done_tidx_r <= cur_tidx;
                    st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
