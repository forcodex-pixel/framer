// ============================================================================
// POE 线程库（线程模板）poe_th_lib_pkg
// ----------------------------------------------------------------------------
// 用途：供 tb_top 后续调用，为 POE_THM 提供线程描述（thread_desc_t）。
//       当前仅建立库文件本身，不改动 poe_types.sv / poe_thm.sv / tb_top.sv。
//
// 数据结构：见 docs/线程数据结构.md（简化版）
//   thread_desc_t = 2245bit：
//     ts_n(5) + grp_n(4) + grp_ts_n(8x5) + ts_id(16x6) + ts_bs_cnt(16x3)
//     + burst(16x4x32) + lock_vld(1) + lock_id(3)
//
// burst 字段按《数据结构位宽总表.md》：
//   iv 视图 burst_iv_t：st/tr/ts_len/branch/burst_type/vld_cu/tsk_id0/1/c0/1/sub_pc0/1
//   c  视图 burst_c_t ：st/tr/rev/burst_type/vld_cu/dma_id0/1/c0/1/occ_ts0/1
//   ts_len/branch 仅 iv 视图字段；c 视图对应位为 rev（置 0）。
//
// 模板生成规则（与本库一致）：
//   1) 组内 ts id 连续，组间边界 ts id 必须跳变（否则应合并为同一组）；
//      组间跳转间隔 1~3 个 id（相邻组首尾 id 差 2~4）；
//   2) 每组首、末 burst 均为 iv_task（保证组末 branch 落在 iv 视图上）；
//   3) branch：非末组的组末收尾 burst = 1，末组 = 0；
//   4) ts_len（iv 视图）= 所属 ts 的 burst 数；
//   5) 每 ts 首个 burst 的 st = 1；
//   6) ts0 固定为 1 个 i_task burst（单任务 iv，vld_cu=0、c0=1）。
//
// 4 个模板：
//   T0：4 组，ts_id={0,3,4,5,7,8,12,13,14}，锁 1 号
//   T1：4 组，ts_id={0,2,5,6,9,10,11}，锁 3 号
//   T2：4 组，ts_id={0,1,4,5,7,10,11}，锁 0 号
//   T3：4 组，ts_id={0,1,2,5,7,8,9,12,13}，锁 7 号
// ============================================================================
package poe_th_lib_pkg;

    import poe_types_pkg::*;

    localparam int TH_TPL_N = 4;     // 模板个数
    localparam int MAX_TS   = 16;    // 每线程最多 ts 数
    localparam int MAX_GRP  = 8;     // 每线程最多 ts 组数
    localparam int MAX_BURST = 4;    // 每 ts 最多 burst 数

    // ---- 线程描述（与 docs/线程数据结构.md 一致，2245bit）----
    typedef struct packed {
        logic [4:0] ts_n;              // 总 ts 数（1..16）
        logic [3:0] grp_n;             // ts 组数（1..8）
        logic [7:0][4:0] grp_ts_n;     // [grp] 每组 ts 数（1..16）
        logic [15:0][5:0] ts_id;       // [ts] 每个 ts 的 id（组内连续）
        logic [15:0][2:0] ts_bs_cnt;   // [ts] 每个 ts 的 burst 数（1..4）
        logic [15:0][3:0][31:0] burst; // [ts][burst]：32bit，iv/c 双类型
        logic lock_vld;                // 是否申请线程锁（1=申请）
        logic [2:0] lock_id;           // 锁资源号（0..7，共 8 个）
    } thread_desc_t;

    // ---- iv burst 构造（burst_iv_t 视图，32bit）----
    function automatic logic [31:0] mk_iv(input logic st,
                                          input logic tr,
                                          input logic [2:0] ts_len,
                                          input logic branch,
                                          input logic vld_cu,
                                          input logic [2:0] tsk0,
                                          input logic c0,
                                          input logic [2:0] tsk1,
                                          input logic c1,
                                          input logic [7:0] sub_pc0,
                                          input logic [7:0] sub_pc1);
        burst_iv_t b;
        b = '0;
        b.st = st;
        b.tr = tr;
        b.ts_len = ts_len;
        b.branch = branch;
        b.burst_type = 1'b0;           // 0 = i/v_task
        b.vld_cu = vld_cu;
        b.tsk_id0 = tsk0;
        b.c0 = c0;
        b.tsk_id1 = tsk1;
        b.c1 = c1;
        b.sub_pc0 = sub_pc0;
        b.sub_pc1 = sub_pc1;
        return 32'(b);
    endfunction

    // ---- c burst 构造（burst_c_t 视图，32bit）----
    function automatic logic [31:0] mk_c(input logic st,
                                         input logic tr,
                                         input logic vld_cu,
                                         input logic [2:0] dma0,
                                         input logic c0,
                                         input logic [2:0] dma1,
                                         input logic c1,
                                         input logic [7:0] occ_ts0,
                                         input logic [7:0] occ_ts1);
        burst_c_t b;
        b = '0;
        b.st = st;
        b.tr = tr;
        b.rev = 4'd0;                  // c 视图保留位（iv 的 ts_len+branch 复用位）
        b.burst_type = 1'b1;           // 1 = c_task
        b.vld_cu = vld_cu;
        b.dma_id0 = dma0;
        b.c0 = c0;
        b.dma_id1 = dma1;
        b.c1 = c1;
        b.occ_ts0 = occ_ts0;
        b.occ_ts1 = occ_ts1;
        return 32'(b);
    endfunction

    // ---- 模板取用：th_tpl(idx)，idx = 0..TH_TPL_N-1 ----
    function automatic thread_desc_t th_tpl(input int idx);
        thread_desc_t t;
        t = '0;
        case (idx)
        // ================= T0：4 组，锁 1 号 =================
        // 组0{0} 组1{3,4,5} 组2{7,8} 组3{12,13,14}（组间差 3/2/4）
        0: begin
            t.ts_n = 5'd9;
            t.grp_n = 4'd4;
            t.grp_ts_n[0] = 5'd1; t.grp_ts_n[1] = 5'd3;
            t.grp_ts_n[2] = 5'd2; t.grp_ts_n[3] = 5'd3;
            t.ts_id[0] = 6'd0;  t.ts_id[1] = 6'd3;  t.ts_id[2] = 6'd4;
            t.ts_id[3] = 6'd5;  t.ts_id[4] = 6'd7;  t.ts_id[5] = 6'd8;
            t.ts_id[6] = 6'd12; t.ts_id[7] = 6'd13; t.ts_id[8] = 6'd14;
            t.ts_bs_cnt[0] = 3'd1; t.ts_bs_cnt[1] = 3'd2; t.ts_bs_cnt[2] = 3'd3;
            t.ts_bs_cnt[3] = 3'd2; t.ts_bs_cnt[4] = 3'd3; t.ts_bs_cnt[5] = 3'd1;
            t.ts_bs_cnt[6] = 3'd2; t.ts_bs_cnt[7] = 3'd2; t.ts_bs_cnt[8] = 3'd1;
            // ts0：单 i_task（组0 唯一 burst，组末跳转）
            t.burst[0][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'h00, 8'h00);
            // ts2：头 iv 双任务 / 尾 c
            t.burst[1][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b1, 3'd0, 1'b1, 3'd3, 1'b1, 8'h10, 8'h20);
            t.burst[1][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd2, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts3：头 c 双任务 / iv / 尾 c
            t.burst[2][0] = mk_c (1'b1, 1'b0, 1'b1, 3'd1, 1'b1, 3'd4, 1'b1, 8'd1, 8'd2);
            t.burst[2][1] = mk_iv(1'b0, 1'b1, 3'd3, 1'b0, 1'b0, 3'd5, 1'b1, 3'd0, 1'b0, 8'h30, 8'h00);
            t.burst[2][2] = mk_c (1'b0, 1'b0, 1'b0, 3'd6, 1'b1, 3'd0, 1'b0, 8'd3, 8'd0);
            // ts4：头 c / 尾 iv（组1 末，branch=1）
            t.burst[3][0] = mk_c (1'b1, 1'b0, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'd2, 8'd0);
            t.burst[3][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b1, 1'b0, 3'd2, 1'b1, 3'd0, 1'b0, 8'h40, 8'h00);
            // ts6：头 iv / c 双任务 / iv 双任务
            t.burst[4][0] = mk_iv(1'b1, 1'b0, 3'd3, 1'b0, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'h50, 8'h00);
            t.burst[4][1] = mk_c (1'b0, 1'b1, 1'b1, 3'd5, 1'b1, 3'd0, 1'b1, 8'd1, 8'd1);
            t.burst[4][2] = mk_iv(1'b0, 1'b0, 3'd3, 1'b0, 1'b1, 3'd6, 1'b1, 3'd7, 1'b1, 8'h60, 8'h70);
            // ts7：单 iv（组2 末，branch=1）
            t.burst[5][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'h80, 8'h00);
            // ts9：头 iv / 尾 c
            t.burst[6][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b0, 3'd0, 1'b1, 3'd0, 1'b0, 8'h90, 8'h00);
            t.burst[6][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd7, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts10：头 c 双任务 / iv
            t.burst[7][0] = mk_c (1'b1, 1'b0, 1'b1, 3'd0, 1'b1, 3'd2, 1'b1, 8'd2, 8'd1);
            t.burst[7][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b0, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'hA0, 8'h00);
            // ts11：单 iv 双任务（末组尾，branch=0）
            t.burst[8][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b1, 3'd5, 1'b1, 3'd6, 1'b1, 8'hB0, 8'hC0);
            t.lock_vld = 1'b1;
            t.lock_id  = 3'd1;
        end
        // ================= T1：4 组，锁 3 号 =================
        // 组0{0} 组1{2} 组2{5,6} 组3{9,10,11}（组间差 2/3/3）
        1: begin
            t.ts_n = 5'd7;
            t.grp_n = 4'd4;
            t.grp_ts_n[0] = 5'd1; t.grp_ts_n[1] = 5'd1;
            t.grp_ts_n[2] = 5'd2; t.grp_ts_n[3] = 5'd3;
            t.ts_id[0] = 6'd0;  t.ts_id[1] = 6'd2;  t.ts_id[2] = 6'd5;
            t.ts_id[3] = 6'd6;  t.ts_id[4] = 6'd9;  t.ts_id[5] = 6'd10;
            t.ts_id[6] = 6'd11;
            t.ts_bs_cnt[0] = 3'd1; t.ts_bs_cnt[1] = 3'd2; t.ts_bs_cnt[2] = 3'd2;
            t.ts_bs_cnt[3] = 3'd1; t.ts_bs_cnt[4] = 3'd3; t.ts_bs_cnt[5] = 3'd2;
            t.ts_bs_cnt[6] = 3'd1;
            // ts0：单 i_task（组0 唯一 burst，branch=1）
            t.burst[0][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'h00, 8'h00);
            // ts2：头/尾均 iv（组1 首=尾，branch=1）
            t.burst[1][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b1, 3'd0, 1'b1, 3'd1, 1'b1, 8'h08, 8'h18);
            t.burst[1][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b1, 1'b0, 3'd2, 1'b1, 3'd0, 1'b0, 8'h28, 8'h00);
            // ts4：头 iv / 尾 c
            t.burst[2][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'h38, 8'h00);
            t.burst[2][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd0, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts5：单 iv（组2 末，branch=1）
            t.burst[3][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'h48, 8'h00);
            // ts7：头 iv 双任务 / c 双任务 / c
            t.burst[4][0] = mk_iv(1'b1, 1'b0, 3'd3, 1'b0, 1'b1, 3'd5, 1'b1, 3'd6, 1'b1, 8'h58, 8'h68);
            t.burst[4][1] = mk_c (1'b0, 1'b0, 1'b1, 3'd1, 1'b1, 3'd3, 1'b1, 8'd1, 8'd2);
            t.burst[4][2] = mk_c (1'b0, 1'b0, 1'b0, 3'd6, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts8：头 c / iv
            t.burst[5][0] = mk_c (1'b1, 1'b0, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'd3, 8'd0);
            t.burst[5][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b0, 1'b0, 3'd7, 1'b1, 3'd0, 1'b0, 8'h78, 8'h00);
            // ts9：单 iv（末组尾，branch=0）
            t.burst[6][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b0, 3'd0, 1'b1, 3'd0, 1'b0, 8'h88, 8'h00);
            t.lock_vld = 1'b1;
            t.lock_id  = 3'd3;
        end
        // ================= T2：4 组，锁 0 号 =================
        // 组0{0,1} 组1{4,5} 组2{7} 组3{10,11}（组间差 3/2/3）
        2: begin
            t.ts_n = 5'd7;
            t.grp_n = 4'd4;
            t.grp_ts_n[0] = 5'd2; t.grp_ts_n[1] = 5'd2;
            t.grp_ts_n[2] = 5'd1; t.grp_ts_n[3] = 5'd2;
            t.ts_id[0] = 6'd0;  t.ts_id[1] = 6'd1;  t.ts_id[2] = 6'd4;
            t.ts_id[3] = 6'd5;  t.ts_id[4] = 6'd7;  t.ts_id[5] = 6'd10;
            t.ts_id[6] = 6'd11;
            t.ts_bs_cnt[0] = 3'd1; t.ts_bs_cnt[1] = 3'd2; t.ts_bs_cnt[2] = 3'd2;
            t.ts_bs_cnt[3] = 3'd3; t.ts_bs_cnt[4] = 3'd1; t.ts_bs_cnt[5] = 3'd2;
            t.ts_bs_cnt[6] = 3'd1;
            // ts0：单 i_task（组0 首，非组末，branch=0）
            t.burst[0][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'h00, 8'h00);
            // ts1：头 iv 双任务 / 尾 iv（组0 末，branch=1）
            t.burst[1][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b1, 3'd0, 1'b1, 3'd3, 1'b1, 8'h10, 8'h20);
            t.burst[1][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b1, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'h30, 8'h00);
            // ts3：头 iv / 尾 c
            t.burst[2][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b0, 3'd5, 1'b1, 3'd0, 1'b0, 8'h40, 8'h00);
            t.burst[2][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'd2, 8'd0);
            // ts4：头 c 双任务 / c / 尾 iv（组1 末，branch=1）
            t.burst[3][0] = mk_c (1'b1, 1'b0, 1'b1, 3'd0, 1'b1, 3'd3, 1'b1, 8'd1, 8'd1);
            t.burst[3][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd5, 1'b1, 3'd0, 1'b0, 8'd2, 8'd0);
            t.burst[3][2] = mk_iv(1'b0, 1'b0, 3'd3, 1'b1, 1'b0, 3'd6, 1'b1, 3'd0, 1'b0, 8'h50, 8'h00);
            // ts6：单 iv 双任务（组2 首=尾，branch=1）
            t.burst[4][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b1, 3'd0, 1'b1, 3'd2, 1'b1, 8'h60, 8'h70);
            // ts8：头 iv / 尾 c 双任务
            t.burst[5][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'h80, 8'h00);
            t.burst[5][1] = mk_c (1'b0, 1'b0, 1'b1, 3'd4, 1'b1, 3'd6, 1'b1, 8'd1, 8'd3);
            // ts9：单 iv（末组尾，branch=0）
            t.burst[6][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b0, 3'd5, 1'b1, 3'd0, 1'b0, 8'h90, 8'h00);
            t.lock_vld = 1'b1;
            t.lock_id  = 3'd0;
        end
        // ================= T3：4 组，锁 7 号 =================
        // 组0{0,1,2} 组1{5} 组2{7,8,9} 组3{12,13}（组间差 3/2/3）
        3: begin
            t.ts_n = 5'd9;
            t.grp_n = 4'd4;
            t.grp_ts_n[0] = 5'd3; t.grp_ts_n[1] = 5'd1;
            t.grp_ts_n[2] = 5'd3; t.grp_ts_n[3] = 5'd2;
            t.ts_id[0] = 6'd0;  t.ts_id[1] = 6'd1;  t.ts_id[2] = 6'd2;
            t.ts_id[3] = 6'd5;  t.ts_id[4] = 6'd7;  t.ts_id[5] = 6'd8;
            t.ts_id[6] = 6'd9;  t.ts_id[7] = 6'd12; t.ts_id[8] = 6'd13;
            t.ts_bs_cnt[0] = 3'd1; t.ts_bs_cnt[1] = 3'd3; t.ts_bs_cnt[2] = 3'd2;
            t.ts_bs_cnt[3] = 3'd1; t.ts_bs_cnt[4] = 3'd2; t.ts_bs_cnt[5] = 3'd2;
            t.ts_bs_cnt[6] = 3'd3; t.ts_bs_cnt[7] = 3'd2; t.ts_bs_cnt[8] = 3'd1;
            // ts0：单 i_task（组0 首，非组末，branch=0）
            t.burst[0][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b0, 3'd1, 1'b1, 3'd0, 1'b0, 8'h00, 8'h00);
            // ts1：头 iv 双任务 / c / c 双任务
            t.burst[1][0] = mk_iv(1'b1, 1'b0, 3'd3, 1'b0, 1'b1, 3'd0, 1'b1, 3'd2, 1'b1, 8'h0A, 8'h1A);
            t.burst[1][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'd2, 8'd0);
            t.burst[1][2] = mk_c (1'b0, 1'b0, 1'b1, 3'd4, 1'b1, 3'd5, 1'b1, 8'd1, 8'd1);
            // ts2：头 c / 尾 iv（组0 末，branch=1）
            t.burst[2][0] = mk_c (1'b1, 1'b0, 1'b0, 3'd6, 1'b1, 3'd0, 1'b0, 8'd3, 8'd0);
            t.burst[2][1] = mk_iv(1'b0, 1'b0, 3'd2, 1'b1, 1'b0, 3'd3, 1'b1, 3'd0, 1'b0, 8'h2A, 8'h00);
            // ts4：单 iv（组1 首=尾，branch=1）
            t.burst[3][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b1, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'h3A, 8'h00);
            // ts6：头 iv 双任务 / 尾 c
            t.burst[4][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b1, 3'd5, 1'b1, 3'd6, 1'b1, 8'h4A, 8'h5A);
            t.burst[4][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd0, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts7：头 c 双任务 / 尾 c
            t.burst[5][0] = mk_c (1'b1, 1'b0, 1'b1, 3'd1, 1'b1, 3'd2, 1'b1, 8'd2, 8'd1);
            t.burst[5][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd7, 1'b1, 3'd0, 1'b0, 8'd1, 8'd0);
            // ts8：头 iv / c / 尾 iv（组2 末，branch=1）
            t.burst[6][0] = mk_iv(1'b1, 1'b0, 3'd3, 1'b0, 1'b0, 3'd0, 1'b1, 3'd0, 1'b0, 8'h6A, 8'h00);
            t.burst[6][1] = mk_c (1'b0, 1'b0, 1'b0, 3'd4, 1'b1, 3'd0, 1'b0, 8'd2, 8'd0);
            t.burst[6][2] = mk_iv(1'b0, 1'b0, 3'd3, 1'b1, 1'b0, 3'd2, 1'b1, 3'd0, 1'b0, 8'h7A, 8'h00);
            // ts10：头 iv / 尾 c 双任务
            t.burst[7][0] = mk_iv(1'b1, 1'b0, 3'd2, 1'b0, 1'b0, 3'd5, 1'b1, 3'd0, 1'b0, 8'h8A, 8'h00);
            t.burst[7][1] = mk_c (1'b0, 1'b0, 1'b1, 3'd3, 1'b1, 3'd6, 1'b1, 8'd1, 8'd2);
            // ts11：单 iv 双任务（末组尾，branch=0）
            t.burst[8][0] = mk_iv(1'b1, 1'b0, 3'd1, 1'b0, 1'b1, 3'd1, 1'b1, 3'd3, 1'b1, 8'h9A, 8'hAA);
            t.lock_vld = 1'b1;
            t.lock_id  = 3'd7;
        end
        default: t = '0;
        endcase
        return t;
    endfunction

endpackage
