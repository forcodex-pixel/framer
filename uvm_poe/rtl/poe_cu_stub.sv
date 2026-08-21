// POE CU 简化桩：从 burst_sch 接收 i/v_task burst（emit_cu_vld/tid/burst），
// 延迟 LATENCY 拍执行完，回 cu_done（含 tid 和 ts 序号 tidx，供 THM 的 cur_ts 推进统计）。
// 占位：真实 CU 按 burst 的 tsk_id0/1 查 CSR.vtsk_c 判断任务是否真实执行，
// 并按 sub_pc0/1 从 I_BUF_B 取子指令；当前桩直接完成。
module poe_cu_stub #(
    parameter int LATENCY = 1
    ) (
    input logic clk,
    input logic rst_n,
    input logic emit_cu_vld,
    input logic [5:0] emit_cu_tid,
    input logic [3:0] emit_cu_tidx,
    input logic [BURST_W-1:0] emit_cu_burst,
    output logic cu_ack,
    output logic cu_done_vld,
    output logic [5:0] cu_done_tid,
    output logic [3:0] cu_done_tidx
    );

    import poe_types_pkg::*;

    logic busy;
    logic [5:0] cur_tid;
    logic [3:0] cur_tidx;
    logic [7:0] remain;
    logic done_vld_r;
    logic [5:0] done_tid_r;
    logic [3:0] done_tidx_r;

    assign cu_ack = !busy;
    assign cu_done_vld = done_vld_r;
    assign cu_done_tid = done_tid_r;
    assign cu_done_tidx = done_tidx_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0;
            cur_tid <= '0;
            cur_tidx <= '0;
            remain <= '0;
            done_vld_r <= 1'b0;
            done_tid_r <= '0;
            done_tidx_r <= '0;
        end else begin
            done_vld_r <= 1'b0;
            if (busy) begin
                if (remain == 1) begin
                    busy <= 1'b0;
                    done_vld_r <= 1'b1;
                    done_tid_r <= cur_tid;
                    done_tidx_r <= cur_tidx;
                end else
                remain <= remain - 1'b1;
            end else if (emit_cu_vld) begin
                busy <= 1'b1;
                cur_tid <= emit_cu_tid;
                cur_tidx <= emit_cu_tidx;
                remain <= LATENCY;
            end
        end
    end
endmodule
