// POE RBA 行为模型：4 个读写机会计数（每 DSE 1 个，对应 dma_ctrl 的 c_task FIFO）
// - 初始 CREDITS（64）个读写机会；dma 每发起 1 次读/写请求，对应计数 -1；
// - 请求后随机 DELAY_MIN~DELAY_MAX（70~75）拍释放读写机会，计数 +1；
// - 计数为 0 时 rdy=0（拉起反压），dma_ctrl 停止从对应 FIFO 出队，
//   FIFO 填满后二级发射被反压。
module poe_rba #(
    parameter int N_CH = 4,
    parameter int CREDITS = 64,
    parameter int DELAY_MIN = 70,
    parameter int DELAY_MAX = 75
) (
    input logic clk,
    input logic rst_n,
    // ---- dma 读/写请求（每 DSE 每拍 ≤1） ----
    input logic [N_CH-1:0] req_vld,
    // ---- 可发起（计数>0）；0=反压 ----
    output logic [N_CH-1:0] rdy,
    // ---- 当前读写机会数（观测用） ----
    output logic [N_CH*7-1:0] cnt
);

    localparam int RET_MAX = CREDITS; // 最大在途请求数 = 初始机会数
    logic [6:0] c [N_CH];
    logic [6:0] ret_q [N_CH][RET_MAX]; // 在途请求剩余释放拍数
    logic [6:0] ret_n [N_CH]; // 在途请求数（0..64）

    assign rdy[0] = (c[0] != 0);
    assign rdy[1] = (c[1] != 0);
    assign rdy[2] = (c[2] != 0);
    assign rdy[3] = (c[3] != 0);
    assign cnt[0*7 +: 7] = c[0];
    assign cnt[1*7 +: 7] = c[1];
    assign cnt[2*7 +: 7] = c[2];
    assign cnt[3*7 +: 7] = c[3];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N_CH; i++) begin
                c[i] <= CREDITS;
                ret_n[i] <= '0;
                for (int j = 0; j < RET_MAX; j++) ret_q[i][j] <= '0;
            end
        end else begin
            for (int i = 0; i < N_CH; i++) begin
                automatic int wr = 0;
                automatic int exp = 0;
                automatic logic issue = req_vld[i] && (c[i] != 0);
                // 1) 在途请求：到期（remaining≤1）释放机会，其余递减并前移压缩
                for (int j = 0; j < ret_n[i]; j++) begin
                    if (ret_q[i][j] <= 7'd1) exp++;
                    else begin
                        ret_q[i][wr] <= ret_q[i][j] - 1'b1;
                        wr++;
                    end
                end
                // 2) 新请求入队（remaining = 70 + rand(0..5)）
                if (issue) begin
                    ret_q[i][wr] <= DELAY_MIN + ($urandom % (DELAY_MAX - DELAY_MIN + 1));
                    wr++;
                end
                ret_n[i] <= wr;
                // 3) 计数：到期 +exp，新请求 -1
                c[i] <= c[i] + exp - (issue ? 7'd1 : 7'd0);
            end
        end
    end
endmodule
