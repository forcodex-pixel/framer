// ============================================================================
// POE 共享数据类型包（THM / th_sch / burst_sch / CU / dma_ctrl / tb 共用）
// 对应《数据结构位宽总表.md》：
//   burst     38bit（一种结构两种类型，burst_type 区分，字段重叠复用）
//             锁字段（仅 ts 首个 burst st=1 时有效）：lock_id(4) + unlock_req(1) + lock_req(1)
//   csr_t     614bit：err/ccr/sys_ts/th_id(6b)/th_stat/o_mes/cur_ts/vtsk_c/dma_c/tw/cw
//   cw_entry  48bit（8×6B/线程，dma_id 索引）
//   c_wnd_entry 168bit（21B，C 窗资源条目）
// C 窗：每线程独享 8 个固定位置（全局号 = tid×8+k，k=0..7），cw 8 项
// （dma_id 0..7）全部固定映射，无共享资源池/FIFO。
// 资源生命周期完全由 c_task 控制（lock/free 成对出现），线程结束不兜底归还。
// ============================================================================
package poe_types_pkg;

    // ---- burst 结构（38b）：一种结构、两种类型，burst_type 区分 ----
    // 两种视图字段不完全相同，按类型复用同一比特位（两个 struct 位布局一致，可互 cast）：
    //  - burst_iv_t（burst_type=0）：tsk_id0/1、sub_pc0/1、ts_len、branch
    //  - burst_c_t （burst_type=1）：dma_id0/1、occ_ts0/1、rev
    // 公共字段：lock_id / unlock_req / lock_req / st / tr / burst_type / vld_cu / c0 / c1
    // 锁字段仅在 ts 首个 burst（st=1）生效：lock_req=1 时该 ts 发射前获取锁；
    // unlock_req=1 时该 ts 完成时释放锁（一段 ts 持锁，由首/末 ts 声明）
    // 位布局：
    //   [37:34] lock_id | [33] unlock_req | [32] lock_req |
    //   [31] st | [30] tr | [29:26] iv{ts_len[2:0],branch} / c{rev} | [25] burst_type |
    //   [24] vld_cu | [23:21] iv{tsk_id0} / c{dma_id0} | [20] c0 |
    //   [19:17] iv{tsk_id1} / c{dma_id1} | [16] c1 |
    //   [15:8] iv{sub_pc0} / c{occ_ts0} | [7:0] iv{sub_pc1} / c{occ_ts1}
    typedef struct packed {
        logic [3:0] lock_id; // 锁 ID（0..15，仅 st=1 有效）
        logic unlock_req; // 解锁请求（仅 st=1 有效，ts 完成时释放锁）
        logic lock_req; // 加锁请求（仅 st=1 有效，ts 首个 burst 发射时获取锁）
        logic st; // 是否为所属 ts 的首个 burst
        logic tr; // 是否涉及 O 窗操作
        logic [2:0] ts_len; // 仅 i/v：所属 ts 包含 burst 数
        logic branch; // 仅 i/v：预留（可能跳转）；真实跳转由模板 ts_id 不连续标记
        logic burst_type; // 0=i/v_task，1=c_task
        logic vld_cu; // 需执行 task 数：0=1 个，1=2 个
        logic [2:0] tsk_id0; // 仅 i/v：任务0 id，查 CSR.vtsk_c
        logic c0; // 任务0 有效标志（0=无效）
        logic [2:0] tsk_id1; // 仅 i/v：任务1 id，查 CSR.vtsk_c
        logic c1; // 任务1 有效标志（0=无效）
        logic [7:0] sub_pc0; // 仅 i/v：task0 指令集指针
        logic [7:0] sub_pc1; // 仅 i/v：task1 指令集指针
    } burst_iv_t;

    typedef struct packed {
        logic [3:0] lock_id; // 锁 ID（0..15，仅 st=1 有效）
        logic unlock_req; // 解锁请求（仅 st=1 有效）
        logic lock_req; // 加锁请求（仅 st=1 有效）
        logic st; // 是否为所属 ts 的首个 burst
        logic tr; // 是否涉及 O 窗操作
        logic [3:0] rev; // 仅 c_task：保留位
        logic burst_type; // 0=i/v_task，1=c_task
        logic vld_cu; // 需执行 task 数：0=1 个，1=2 个
        logic [2:0] dma_id0; // 仅 c_task：任务0 id，查 CSR.dma_c
        logic c0; // 任务0 有效标志（0=无效）
        logic [2:0] dma_id1; // 仅 c_task：任务1 id，查 CSR.dma_c
        logic c1; // 任务1 有效标志（0=无效）
        logic [7:0] occ_ts0; // 仅 c_task：task0 占据的 ts 数
        logic [7:0] occ_ts1; // 仅 c_task：task1 占据的 ts 数
    } burst_c_t;

    localparam int BURST_W = 38;

    // ---- CSR 表项（61B = 488bit）：建线程时同步生成 ----
    typedef struct packed {
        logic [7:0] err; // 1B：线程错误指示（默认 0，刷新条件待定）
        logic [63:0] ccr; // 8B：待定
        logic [47:0] sys_ts; // 6B：系统时戳（线程启动时间）
        logic [5:0] th_id; // 6bit：线程 id（64 线程）
        logic [7:0] th_stat; // 1B：线程状态（IDLE/READY/ISSUED/DONE）
        logic [7:0] o_mes; // 1B：O 窗操作/状态指示（语义待定）
        logic [7:0] cur_ts; // 1B：当前 ts（与 THM cur_ts 同步）
        logic [7:0] vtsk_c; // 1B：i/v 任务执行掩码（tsk_id 查询）
        logic [7:0] dma_c; // 1B：c_task 执行掩码（dma_id 查询，bit i 对应 cw[i]）
        logic [63:0] tw; // 8*1B：待定
        logic [8*48-1:0] cw; // 8×6B：c_task 操作表（dma_id 索引，条目见 cw_entry_t）
    } csr_t;

    // ---- CSR.cw 条目（6B = 48bit） ----
    typedef struct packed {
        logic [19:0] tag; // smc 地址（= op_addr）
        logic op_type; // loc/free：0=loc（申请），1=free（释放）
        logic r; // c_line 有效（smc 读出刷新 c 窗 c_line 后写 1）
        logic o; // 1=该线程占据该 c 窗资源
        logic [7:0] c_line_num; // C 窗位置（线程内索引 = dma_id，0..7；全局位置 = tid×8+dma_id，内部 9bit 计算）
        logic [7:0] start_ts; // 起始 ts
        logic [7:0] occ_ts; // 占据 ts 数
        logic rsv; // 保留
    } cw_entry_t;

    // ---- C 窗资源条目（168bit = 21B） ----
    typedef struct packed {
        logic [19:0] tag; // smc 地址
        logic [127:0] c_line; // 行数据（free 时经 RBA 写 SMC[tag]）
        logic d; // dirty：申请后 0，CU 改写 c_line 后 1
        logic o; // 占用标志：申请到资源后写 1
        logic r; // c_line 有效：smc 读出刷新 c 窗 c_line 后写 1
        logic [8:0] cnt; // 老化计数：超上限强制释放（o=0，资源号入 FIFO）
        logic [7:0] ind; // C 窗位置（线程内索引 = dma_id，0..7；全局位置 = tid×8+dma_id，内部 9bit 计算）
    } c_wnd_entry_t;

    localparam int CSR_W = 8+64+48+6+8+8+8+8+8+64+384;
    localparam int CW_ENTRY_W = 48;
    localparam int C_WND_ENTRY_W = 168;

endpackage
