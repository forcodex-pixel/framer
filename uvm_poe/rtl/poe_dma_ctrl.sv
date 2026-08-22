// POE dma_ctrl 行为模型：4 个 c_task FIFO（每 DSE 1 个），仅作存储缓冲。
// - 二级发射（DSE 调度器）发射的 c_task 推入对应 FIFO；**FIFO 满时反压二级发射**
//   （burst_sch 不发射该 DSE 的 c_task，槽保留在 c_task 缓存，下拍重选）；
// - 每个 FIFO 每拍出队 1 个（模型化 dma_ctrl 消化 c_task，腾出空间）；
// - c_task 的 done 已由二级发射发射时返回（发射即完成），本模块不回 done。
module poe_dma_ctrl #(
    parameter int FIFO_DEPTH = 8
) (
    input logic clk,
    input logic rst_n,
    // ---- burst_sch：4 路 push（每 DSE 1 个，存 c_task） ----
    input logic [3:0] push_vld,
    input logic [23:0] push_tid,
    input logic [15:0] push_tidx,
    input logic [11:0] push_dma_id,
    input logic [7:0] push_tag,
    input logic [3:0] push_op,
    // ---- 满状态（→ burst_sch 反压） ----
    output logic [3:0] fifo_full
);

    import poe_types_pkg::*;

    // ---- c_task FIFO 项：{tid(6), tidx(4), dma_id(3), tag(2), op(1)} ----
    typedef struct packed {
        logic [5:0] tid;
        logic [3:0] tidx;
        logic [2:0] dma_id;
        logic [1:0] tag;
        logic op;
    } ct_fifo_item_t;

    localparam int PT_W = $clog2(FIFO_DEPTH);
    ct_fifo_item_t f_mem [4][FIFO_DEPTH];
    logic [PT_W-1:0] f_head [4];
    logic [PT_W-1:0] f_tail [4];
    logic [PT_W:0] f_cnt [4];

    assign fifo_full[0] = (f_cnt[0] == FIFO_DEPTH);
    assign fifo_full[1] = (f_cnt[1] == FIFO_DEPTH);
    assign fifo_full[2] = (f_cnt[2] == FIFO_DEPTH);
    assign fifo_full[3] = (f_cnt[3] == FIFO_DEPTH);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int d = 0; d < 4; d++) begin
                for (int i = 0; i < FIFO_DEPTH; i++) f_mem[d][i] <= '0;
                f_head[d] <= '0;
                f_tail[d] <= '0;
                f_cnt[d] <= '0;
            end
        end else begin
            for (int d = 0; d < 4; d++) begin
                automatic logic push = push_vld[d] && !fifo_full[d];
                automatic logic pop = (f_cnt[d] != 0);
                // 推入（满时不收，burst_sch 侧已按 fifo_full 反压）
                if (push) begin
                    f_mem[d][f_tail[d]] <= {push_tid[d*6 +: 6],
                                            push_tidx[d*4 +: 4],
                                            push_dma_id[d*3 +: 3],
                                            push_tag[d*2 +: 2],
                                            push_op[d]};
                    f_tail[d] <= (f_tail[d] == FIFO_DEPTH-1) ? '0 : f_tail[d] + 1'b1;
                end
                // 出队（模型化消化，每拍 1 个）
                if (pop)
                    f_head[d] <= (f_head[d] == FIFO_DEPTH-1) ? '0 : f_head[d] + 1'b1;
                // 计数：写 +1 / 读 -1（同拍写读净 0）
                if (push && !pop) f_cnt[d] <= f_cnt[d] + 1'b1;
                else if (!push && pop) f_cnt[d] <= f_cnt[d] - 1'b1;
            end
        end
    end
endmodule
