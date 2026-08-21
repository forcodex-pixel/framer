// koa_if：KOA 接口（5 条 fgOTN/X2X 流带 cid/pos + 2 条串口流 + KO 输出）
// 多平面流为 packed 拼接：平面 0 在低位
interface koa_if #(parameter int NUM_OH_PLANES = 4,
    parameter int NUM_X2X_PLANES = 8,
    parameter int CID_W = 17,
    parameter int POS_W = 3)
    (input logic clk, input logic rst_n);

    // ---- fgOTN：OH_EXT / OH_INS（各 4 平面） ----
    logic [NUM_OH_PLANES-1:0] oh_e_vld;
    logic [NUM_OH_PLANES*3-1:0] oh_e_pri;
    logic [NUM_OH_PLANES*CID_W-1:0] oh_e_cid;
    logic [NUM_OH_PLANES*POS_W-1:0] oh_e_pos;
    logic [NUM_OH_PLANES-1:0] oh_e_rdy;
    logic [NUM_OH_PLANES-1:0] oh_i_vld;
    logic [NUM_OH_PLANES*3-1:0] oh_i_pri;
    logic [NUM_OH_PLANES*CID_W-1:0] oh_i_cid;
    logic [NUM_OH_PLANES*POS_W-1:0] oh_i_pos;
    logic [NUM_OH_PLANES-1:0] oh_i_rdy;
    // ---- X2X：APS_EXT / APS_INS / ALM（各 8 平面） ----
    logic [NUM_X2X_PLANES-1:0] aps_e_vld;
    logic [NUM_X2X_PLANES*3-1:0] aps_e_pri;
    logic [NUM_X2X_PLANES*CID_W-1:0] aps_e_cid;
    logic [NUM_X2X_PLANES*POS_W-1:0] aps_e_pos;
    logic [NUM_X2X_PLANES-1:0] aps_e_rdy;
    logic [NUM_X2X_PLANES-1:0] aps_i_vld;
    logic [NUM_X2X_PLANES*3-1:0] aps_i_pri;
    logic [NUM_X2X_PLANES*CID_W-1:0] aps_i_cid;
    logic [NUM_X2X_PLANES*POS_W-1:0] aps_i_pos;
    logic [NUM_X2X_PLANES-1:0] aps_i_rdy;
    logic [NUM_X2X_PLANES-1:0] alm_vld;
    logic [NUM_X2X_PLANES*3-1:0] alm_pri;
    logic [NUM_X2X_PLANES*CID_W-1:0] alm_cid;
    logic [NUM_X2X_PLANES*POS_W-1:0] alm_pos;
    logic [NUM_X2X_PLANES-1:0] alm_rdy;
    // ---- 串口：UART_EXT / UART_INS（各 1 路） ----
    logic u_e_vld;
    logic [2:0] u_e_pri;
    logic u_e_rdy;
    logic u_i_vld;
    logic [2:0] u_i_pri;
    logic u_i_rdy;
    // ---- KO 输出 ----
    logic out_vld;
    logic [2:0] out_pri;
    logic [2:0] out_src; // 优先级组号 0..7
    logic [2:0] out_stream; // 来源流 0..6
    logic [CID_W-1:0] out_cid;
    logic [POS_W-1:0] out_pos;
    // ---- 预读接口（随 KO 报文：KOA 缓冲后与报文对齐输出） ----
    logic [3:0] ko_pre_vld; // 入口：4 组预读指示
    logic [79:0] ko_dma_addr; // 入口：4×20bit smc 地址
    logic [3:0] ko_pre_op; // 入口：4×1bit 操作类型（0=loc 1=free）
    logic [3:0] out_pre_vld; // 出口：随 out_vld 对齐
    logic [79:0] out_dma_addr;
    logic [3:0] out_pre_op;

    clocking drv_cb @(posedge clk);
        default input #1step output #1;
        output oh_e_vld, oh_e_pri, oh_e_cid, oh_e_pos,
        oh_i_vld, oh_i_pri, oh_i_cid, oh_i_pos,
        aps_e_vld, aps_e_pri, aps_e_cid, aps_e_pos,
        aps_i_vld, aps_i_pri, aps_i_cid, aps_i_pos,
        alm_vld, alm_pri, alm_cid, alm_pos,
        u_e_vld, u_e_pri,
        u_i_vld, u_i_pri;
        input oh_e_rdy, oh_i_rdy, aps_e_rdy, aps_i_rdy, alm_rdy, u_e_rdy, u_i_rdy,
        out_vld, out_pri, out_src, out_stream, out_cid, out_pos,
        out_pre_vld, out_dma_addr, out_pre_op;
    endclocking

    clocking mon_cb @(posedge clk);
        default input #1step;
        input oh_e_vld, oh_e_pri, oh_e_rdy,
        oh_e_cid, oh_e_pos,
        oh_i_vld, oh_i_pri, oh_i_rdy, oh_i_cid, oh_i_pos,
        aps_e_vld, aps_e_pri, aps_e_rdy, aps_e_cid, aps_e_pos,
        aps_i_vld, aps_i_pri, aps_i_rdy, aps_i_cid, aps_i_pos,
        alm_vld, alm_pri, alm_rdy, alm_cid, alm_pos,
        u_e_vld, u_e_pri, u_e_rdy,
        u_i_vld, u_i_pri, u_i_rdy,
        out_vld, out_pri, out_src, out_stream, out_cid, out_pos,
        out_pre_vld, out_dma_addr, out_pre_op;
    endclocking
endinterface
