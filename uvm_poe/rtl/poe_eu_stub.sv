// POE EU 简化桩：1 个 EU 内含 4 个 CU 桩（task0→CU0、task1→CU1，CU2/CU3 预留），
// burst 的 ≤2 个 i/v task 分发给 CU 桩执行，全部完成后聚合回 1 个 eu_done（按 burst 计）。
// 占位：真实 CU 按 tsk_id 查 CSR.vtsk_c、按 sub_pc 取子指令；当前桩直接按 LATENCY 完成。
module poe_eu_stub #(
    parameter int LATENCY = 1
    ) (
    input logic clk,
    input logic rst_n,
    input logic emit_eu_vld,
    input logic [5:0] emit_eu_tid,
    input logic [3:0] emit_eu_tidx,
    input logic [BURST_W-1:0] emit_eu_burst,
    output logic eu_ack,
    output logic eu_done_vld,
    output logic [5:0] eu_done_tid,
    output logic [3:0] eu_done_tidx
    );

    import poe_types_pkg::*;

    // ---- 4 个 CU 桩槽位（本 burst 只使用 CU0/CU1，CU2/CU3 预留） ----
    logic [3:0] sl_busy;
    logic [3:0][7:0] sl_remain;
    logic [1:0] act; // 本 burst 实际分发的 task 掩码（bit0=task0，bit1=task1）
    logic [5:0] cur_tid;
    logic [3:0] cur_tidx;
    logic done_vld_r;
    logic [5:0] done_tid_r;
    logic [3:0] done_tidx_r;

    assign eu_ack = !(sl_busy[0] || sl_busy[1]);
    assign eu_done_vld = done_vld_r;
    assign eu_done_tid = done_tid_r;
    assign eu_done_tidx = done_tidx_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sl_busy <= '0;
            sl_remain <= '0;
            act <= 2'd0;
            cur_tid <= '0;
            cur_tidx <= '0;
            done_vld_r <= 1'b0;
            done_tid_r <= '0;
            done_tidx_r <= '0;
        end else begin
            automatic logic [1:0] dm = 2'd0;
            done_vld_r <= 1'b0;
            if (emit_eu_vld && eu_ack) begin
                automatic burst_iv_t b = emit_eu_burst;
                act <= {b.vld_cu && b.c1, b.c0};
                sl_busy[0] <= b.c0;
                sl_busy[1] <= b.vld_cu && b.c1;
                sl_remain[0] <= LATENCY;
                sl_remain[1] <= LATENCY;
                cur_tid <= emit_eu_tid;
                cur_tidx <= emit_eu_tidx;
            end
            // CU 桩流水：倒计时完成后清 busy
            for (int s = 0; s < 2; s++) begin
                if (sl_busy[s]) begin
                    if (sl_remain[s] <= 8'd1) begin
                        sl_busy[s] <= 1'b0;
                        dm[s] = 1'b1;
                    end else
                    sl_remain[s] <= sl_remain[s] - 1'b1;
                end
            end
            // 聚合：本 burst 分发的 task 全部完成 → 回 1 个 done（按 burst 计）
            if ((dm == act) && (act != 2'd0)) begin
                done_vld_r <= 1'b1;
                done_tid_r <= cur_tid;
                done_tidx_r <= cur_tidx;
            end
        end
    end
endmodule
