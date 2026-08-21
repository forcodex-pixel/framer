// ============================================================================
// KOA 行为模型 v6：5×SBUF + 同段合并向量写 + RR+SP 调度
// 结构（对应方案图）：
// - 5 个独立 SBUF（EXT/INS/ALM/UART_EXT/UART_INS，深度 SBUF_DEPTH=2560），
//   每个 SBUF 的存储按优先级拆成 8 个队列段（pri 0..7，0 最高，每段 320 深）：
//     EXT_SBUF      <- OH_EXT + APS_EXT
//     INS_SBUF      <- OH_INS + APS_INS
//     ALM_SBUF      <- ALM
//     UART_EXT_SBUF <- UART_EXT
//     UART_INS_SBUF <- UART_INS
// - 写入（同段合并向量写）：业务源 vld 直写对应段，不做跨流仲裁；
//   同段同拍可合并写多条（MAX_WR_SEG 上限），写入顺序固定为
//   APS 类（编小优先）→ OH 类（编小优先）→ UART，保持"APS 固定靠前"；
//   段剩余空间不足时，排在后面的候选让位（rdy=0、vld 保持，不丢报文）。
//   每段每拍实际写入条数 = wr_cnt[s][g]，tail 按条数推进，cnt 按条数累加。
// - 读（每拍 1 条）：先按优先级高低选组（SP，组号 0..7），组内对 5 个 SBUF
//   按 rr_ptr（每拍推进）轮询，取最先非空的队列段出队。
// - 输出：out_vld + 48B + out_pri(组号) + out_src(组号) + out_stream/cid/pos
//   （保序键 stream+cid+pos 供 THM 使用；pri 随路传入决定段）。
// ============================================================================
module koa #(
    parameter int NUM_OH_PLANES = 4, // fgOTN 开销流平面数（OH_EXT/OH_INS 各 N）
    parameter int NUM_X2X_PLANES = 8, // X2X 平面数（APS_EXT/APS_INS/ALM 各 N）
    parameter int CID_W = 17, // 通道号位宽（1 时隙粒度）
    parameter int POS_W = 3, // 开销位置位宽（OH 0..7 / APS 0 / ALM 0..3）
    parameter int SBUF_DEPTH = 2560, // 每个 SBUF 总深度（地址拆成 8 个 pri 段）
    parameter int MAX_WR_SEG = 16 // 每段每拍合并写入条数上限（8 APS + 4 OH 同段最坏 12，留余量）
    ) (
    input logic clk,
    input logic rst_n,
    // ---- fgOTN：OH_EXT / OH_INS（开销提取 / 下插，各 NUM_OH_PLANES 平面）----
    input logic [NUM_OH_PLANES-1:0] oh_e_vld,
    input logic [NUM_OH_PLANES*3-1:0] oh_e_pri,
    input logic [NUM_OH_PLANES*CID_W-1:0] oh_e_cid,
    input logic [NUM_OH_PLANES*POS_W-1:0] oh_e_pos,
    output logic [NUM_OH_PLANES-1:0] oh_e_rdy,
    input logic [NUM_OH_PLANES-1:0] oh_i_vld,
    input logic [NUM_OH_PLANES*3-1:0] oh_i_pri,
    input logic [NUM_OH_PLANES*CID_W-1:0] oh_i_cid,
    input logic [NUM_OH_PLANES*POS_W-1:0] oh_i_pos,
    output logic [NUM_OH_PLANES-1:0] oh_i_rdy,
    // ---- X2X：APS_EXT / APS_INS / ALM（各 NUM_X2X_PLANES 平面）----
    input logic [NUM_X2X_PLANES-1:0] aps_e_vld,
    input logic [NUM_X2X_PLANES*3-1:0] aps_e_pri,
    input logic [NUM_X2X_PLANES*CID_W-1:0] aps_e_cid,
    input logic [NUM_X2X_PLANES*POS_W-1:0] aps_e_pos,
    output logic [NUM_X2X_PLANES-1:0] aps_e_rdy,
    input logic [NUM_X2X_PLANES-1:0] aps_i_vld,
    input logic [NUM_X2X_PLANES*3-1:0] aps_i_pri,
    input logic [NUM_X2X_PLANES*CID_W-1:0] aps_i_cid,
    input logic [NUM_X2X_PLANES*POS_W-1:0] aps_i_pos,
    output logic [NUM_X2X_PLANES-1:0] aps_i_rdy,
    input logic [NUM_X2X_PLANES-1:0] alm_vld,
    input logic [NUM_X2X_PLANES*3-1:0] alm_pri,
    input logic [NUM_X2X_PLANES*CID_W-1:0] alm_cid,
    input logic [NUM_X2X_PLANES*POS_W-1:0] alm_pos,
    output logic [NUM_X2X_PLANES-1:0] alm_rdy,
    // ---- 串口：UART_EXT / UART_INS（无 cid/pos，直连）----
    input logic u_e_vld,
    input logic [2:0] u_e_pri,
    output logic u_e_rdy,
    input logic u_i_vld,
    input logic [2:0] u_i_pri,
    output logic u_i_rdy,
    // ---- KO 输出（→ THM，寄存一拍）----
    output logic out_vld,
    output logic [2:0] out_pri,
    output logic [2:0] out_src,
    output logic [2:0] out_stream,
    output logic [CID_W-1:0] out_cid,
    output logic [POS_W-1:0] out_pos,
    // ---- 预读接口（随 KO 报文：输入随拍写入 SBUF，输出与报文对齐）----
    input logic [3:0] ko_pre_vld, // 4 组预读指示
    input logic [79:0] ko_dma_addr, // 4×20bit smc 地址
    input logic [3:0] ko_pre_op, // 4×1bit 操作类型
    output logic [3:0] out_pre_vld,
    output logic [79:0] out_dma_addr,
    output logic [3:0] out_pre_op
    );

    // SBUF 编号：0=EXT 1=INS 2=ALM 3=UART_EXT 4=UART_INS
    localparam int N_SBUF = 5;
    localparam int PRI_Q_DEPTH = SBUF_DEPTH / 8; // 每个 pri 段深度（=320）
    localparam int PTR_W = $clog2(PRI_Q_DEPTH);
    localparam int WR_CNT_W = $clog2(MAX_WR_SEG + 1); // 段内写入条数计数位宽
    // 条目 {pre_vld(4), dma_addr(80), pre_op(4), stream, cid, pos}
    localparam int PRE_W = 4 + 80 + 4;
    localparam int PKG_W = PRE_W + 3 + CID_W + POS_W;

    // ---- 5×SBUF × 8 段存储（每段独立 FIFO）----
    logic [PKG_W-1:0] sbuf_mem [N_SBUF][8][PRI_Q_DEPTH];
    logic [PTR_W-1:0] sbuf_head [N_SBUF][8];
    logic [PTR_W-1:0] sbuf_tail [N_SBUF][8];
    logic [PTR_W:0] sbuf_cnt [N_SBUF][8];

    // ---- 写入接受（组合）：同段同拍可合并写多条（合并向量写）----
    // 段号 = 报文 pri（0..7，随路传入）；rdy = 对应平面本拍被接受（空间不足时让位）
    logic [NUM_OH_PLANES-1:0] oh_e_acc, oh_i_acc;
    logic [NUM_X2X_PLANES-1:0] aps_e_acc, aps_i_acc, alm_acc;
    logic u_e_acc, u_i_acc;

    // 本拍各段待写列表：wr_pkg[s][g][0 .. wr_cnt-1]，写入顺序即"DUT 出队前的段内顺序"
    logic [PKG_W-1:0] wr_pkg [N_SBUF][8][MAX_WR_SEG];
    logic [WR_CNT_W-1:0] wr_cnt [N_SBUF][8];

    always_comb begin
        // 段内已排入条数累加器（块内 automatic，避免组合环）
        automatic logic [WR_CNT_W-1:0] n_acc [N_SBUF][8];
        automatic logic [PKG_W-1:0] tmp [N_SBUF][8][MAX_WR_SEG];
        for (int s = 0; s < N_SBUF; s++)
            for (int g = 0; g < 8; g++) n_acc[s][g] = '0;

        // APS 类固定靠前：APS_EXT / APS_INS / ALM（同段同 pri 多平面全部可写，编小优先顺序）
        for (int i = 0; i < NUM_X2X_PLANES; i++) begin
            automatic logic [2:0] g = aps_e_pri[i*3 +: 3];
            aps_e_acc[i] = 1'b0;
            if (aps_e_vld[i] && (sbuf_cnt[0][g] + n_acc[0][g] < PRI_Q_DEPTH)) begin
                tmp[0][g][n_acc[0][g]] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                          3'd2, aps_e_cid[i*CID_W +: CID_W],
                                          aps_e_pos[i*POS_W +: POS_W]};
                n_acc[0][g] = n_acc[0][g] + 1'b1;
                aps_e_acc[i] = 1'b1;
            end
            g = aps_i_pri[i*3 +: 3];
            aps_i_acc[i] = 1'b0;
            if (aps_i_vld[i] && (sbuf_cnt[1][g] + n_acc[1][g] < PRI_Q_DEPTH)) begin
                tmp[1][g][n_acc[1][g]] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                          3'd3, aps_i_cid[i*CID_W +: CID_W],
                                          aps_i_pos[i*POS_W +: POS_W]};
                n_acc[1][g] = n_acc[1][g] + 1'b1;
                aps_i_acc[i] = 1'b1;
            end
            g = alm_pri[i*3 +: 3];
            alm_acc[i] = 1'b0;
            if (alm_vld[i] && (sbuf_cnt[2][g] + n_acc[2][g] < PRI_Q_DEPTH)) begin
                tmp[2][g][n_acc[2][g]] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                          3'd4, alm_cid[i*CID_W +: CID_W],
                                          alm_pos[i*POS_W +: POS_W]};
                n_acc[2][g] = n_acc[2][g] + 1'b1;
                alm_acc[i] = 1'b1;
            end
        end
        // OH 类排在 APS 之后：OH_EXT / OH_INS（同段同 pri 多平面全部可写，编小优先顺序）
        for (int i = 0; i < NUM_OH_PLANES; i++) begin
            automatic logic [2:0] g = oh_e_pri[i*3 +: 3];
            oh_e_acc[i] = 1'b0;
            if (oh_e_vld[i] && (sbuf_cnt[0][g] + n_acc[0][g] < PRI_Q_DEPTH)) begin
                tmp[0][g][n_acc[0][g]] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                          3'd0, oh_e_cid[i*CID_W +: CID_W],
                                          oh_e_pos[i*POS_W +: POS_W]};
                n_acc[0][g] = n_acc[0][g] + 1'b1;
                oh_e_acc[i] = 1'b1;
            end
            g = oh_i_pri[i*3 +: 3];
            oh_i_acc[i] = 1'b0;
            if (oh_i_vld[i] && (sbuf_cnt[1][g] + n_acc[1][g] < PRI_Q_DEPTH)) begin
                tmp[1][g][n_acc[1][g]] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                          3'd1, oh_i_cid[i*CID_W +: CID_W],
                                          oh_i_pos[i*POS_W +: POS_W]};
                n_acc[1][g] = n_acc[1][g] + 1'b1;
                oh_i_acc[i] = 1'b1;
            end
        end
        // UART（每段最多 1 条）
        u_e_acc = u_e_vld && (sbuf_cnt[3][u_e_pri] < PRI_Q_DEPTH);
        u_i_acc = u_i_vld && (sbuf_cnt[4][u_i_pri] < PRI_Q_DEPTH);
        if (u_e_acc) begin
                tmp[3][u_e_pri][0] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                      3'd5, '0, 3'd0};
            n_acc[3][u_e_pri] = 1'b1;
        end
        if (u_i_acc) begin
                tmp[4][u_i_pri][0] = {ko_pre_vld, ko_dma_addr, ko_pre_op,
                                      3'd6, '0, 3'd0};
            n_acc[4][u_i_pri] = 1'b1;
        end
        // 拷贝到模块信号
        for (int s = 0; s < N_SBUF; s++)
        for (int g = 0; g < 8; g++) begin
            wr_cnt[s][g] = n_acc[s][g];
            for (int n = 0; n < MAX_WR_SEG; n++)
                wr_pkg[s][g][n] = tmp[s][g][n];
        end
    end
    // rdy = 本拍接受（组合，随 vld/段剩余空间变化）
    assign oh_e_rdy = oh_e_acc;
    assign oh_i_rdy = oh_i_acc;
    assign aps_e_rdy = aps_e_acc;
    assign aps_i_rdy = aps_i_acc;
    assign alm_rdy = alm_acc;
    assign u_e_rdy = u_e_acc;
    assign u_i_rdy = u_i_acc;

    // ---- 读侧：SP 选最高非空 pri 组（组号最小），组内 5 SBUF 轮询（rr_ptr 每拍推进）----
    logic [2:0] rr_ptr;
    logic [2:0] sel_grp;
    logic [2:0] sel_sbuf;
    logic rd_valid;
    always_comb begin
        sel_grp = 3'd7;
        rd_valid = 1'b0;
        for (int g = 0; g < 8; g++) begin
            for (int s = 0; s < N_SBUF; s++)
            if (sbuf_cnt[s][g] != 0) begin
                sel_grp = g[2:0];
                rd_valid = 1'b1;
                break;
            end
            if (rd_valid) break;
        end
        sel_sbuf = rr_ptr;
        if (rd_valid)
            for (int k = 0; k < N_SBUF; k++)
            if (sbuf_cnt[(rr_ptr + k) % N_SBUF][sel_grp] != 0) begin
            sel_sbuf = (rr_ptr + k) % N_SBUF;
            break;
        end
    end

    // ---- 主时序：合并向量写（按段）+ SP/RR 出队 + 输出寄存（一拍）----
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < N_SBUF; s++)
            for (int g = 0; g < 8; g++) begin
                sbuf_head[s][g] <= '0;
                sbuf_tail[s][g] <= '0;
                sbuf_cnt[s][g] <= '0;
            end
            rr_ptr <= '0;
            out_vld <= 1'b0;
            out_pri <= 3'd0;
            out_src <= 3'd0;
        out_stream <= 3'd0;
        out_cid <= '0;
        out_pos <= '0;
        out_pre_vld <= 4'd0;
        out_dma_addr <= '0;
        out_pre_op <= 4'd0;
        end else begin
            // ---- 写入：按段合并写 wr_cnt 条（地址 tail+n，tail 按条数推进）----
            for (int s = 0; s < N_SBUF; s++)
                for (int g = 0; g < 8; g++)
                if (wr_cnt[s][g] != 0) begin
                automatic logic [PTR_W:0] tail_n = sbuf_tail[s][g] + wr_cnt[s][g];
                for (int n = 0; n < MAX_WR_SEG; n++)
                if (n < wr_cnt[s][g]) begin
                    automatic logic [PTR_W:0] wa_n = sbuf_tail[s][g] + n[PTR_W-1:0];
                    sbuf_mem[s][g][(wa_n >= PRI_Q_DEPTH) ? wa_n - PRI_Q_DEPTH
                    : wa_n[PTR_W-1:0]] <= wr_pkg[s][g][n];
                end
                sbuf_tail[s][g] <= (tail_n >= PRI_Q_DEPTH) ? tail_n - PRI_Q_DEPTH
                : tail_n[PTR_W-1:0];
            end

            // ---- 读：SP/RR 选中段出队（head + out 拆包，用沿前状态）----
            if (rd_valid) begin
                sbuf_head[sel_sbuf][sel_grp] <= (sbuf_head[sel_sbuf][sel_grp] == PRI_Q_DEPTH-1)
                ? '0 : sbuf_head[sel_sbuf][sel_grp] + 1'b1;
                out_vld <= 1'b1;
                out_pri <= sel_grp;
                out_src <= sel_grp;
        out_pre_vld <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-1 -: 4];
        out_dma_addr <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-5 -: 80];
        out_pre_op <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-85 -: 4];
        out_stream <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-89 -: 3];
        out_cid <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-92 -: CID_W];
        out_pos <= sbuf_mem[sel_sbuf][sel_grp][sbuf_head[sel_sbuf][sel_grp]][PKG_W-92-CID_W -: POS_W];
            end else begin
                out_vld <= 1'b0;
            end

            // ---- cnt 合并更新：写 +wr_cnt / 读 -1（同段同拍写读按条数净算）----
            for (int s = 0; s < N_SBUF; s++)
                for (int g = 0; g < 8; g++)
                    sbuf_cnt[s][g] <= sbuf_cnt[s][g] + wr_cnt[s][g]
                    - ((rd_valid && (sel_sbuf == s) && (sel_grp == g)) ? 1'b1 : 1'b0);

            // rr 指针每拍推进（同优先级 5 路轮询）
            rr_ptr <= (rr_ptr == N_SBUF-1) ? '0 : rr_ptr + 1'b1;
        end
    end
endmodule
