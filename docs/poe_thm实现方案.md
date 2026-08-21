# POE THM（线程管理器）实现方案详解

> 本文档以 `uvm_poe/rtl/poe_thm.sv` 当前实现为准（v5 行为模型），用于代码走查与方案核对。
> 早期草案见《THM设计方案.md》（32 线程 / IFU 双缓冲预取 / i_rdy），本文档是与之对应的
> **当前代码实现**：64 线程、无独立 IFU 实体（线程描述由 tb_top 旁路直接给定）、
> ts 编号可跳转、每 ts 独立 burst 模式。

## 1. 模块定位与设计要点

THM 是 KOA → POE 链路的枢纽：接收 KO 报文与线程描述，按 `(stream, cid, pos)` 保序建线程，
为 th_sch 提供可发射 burst（一级发射），并根据 CU/dma_ctrl 完成反馈推进当前 ts。

核心设计点（当前代码语义）：

- **线程 = 若干 ts，每个 ts = 若干 burst**：burst 为 32bit 一种结构两种类型
  （`burst_iv_t` / `burst_c_t`，`burst_type` 区分，见 poe_types_pkg）。
- **ts 编号可跳转**：线程的 ts 是一组递增编号（如 0,1,5,6,10…），`th_cur_ts` 按编号推进
  （模拟 ts 跳转），不再假定 ts 连续。
- **每 ts 独立 burst 数**：`th_bs_cnt` 为每 ts 一个 3bit 字段；ts0 固定 1 条，其余 1..4 条；
  每个 burst 独立随机类型（i/v 或 c）。
- **保序在 THM**：同 key（stream+cid+pos）已有活跃线程时，新报文存入 8 深缓存等待，
  前序线程释放后按 FIFO 放行；缓存满或线程满时 `ko_rdy=0` 反压。
- **一级发射语义**：READY 线程按 (pri, tid) 被 th_sch 选中发射（每拍 ≤2），进入 ISSUED 打拍；
  打拍结束后 bs_pc+1 回 READY。**READY ≠ 回 IDLE**，burst 队列可含当前 ts 及更靠后 ts 的 burst。
- **cur_ts 只由完成反馈推进**：`cu_done` / `dma_done` 统计当前 ts 已完成 burst 数，
  达到 `th_need` 后 `th_ts_idx+1` 并跳转到下一个 ts 编号；末 ts 完成则线程 T_DONE → 释放。
- **pre_read 预读**：KO 输入可携带最多 **4 组预读信息**（入口 `{pre_read, dma_addr, op_type}`）；
  带预读的 KO **照常建线程/走保序**，**线程创建的同拍**把预读请求转发给二级发射
  （出口 `{vld, tid, dma_addr, op_type}`×4，tid=新建线程，格式与 dma_ctrl 入口一致）；
  二级发射用 **8 深预读缓存**单独保存，**最高优先级调度**，缓存满时反压
  （限制带预读请求的 KO 报文创建线程）。
- **CSR 同步生成**：建线程时同步写 `csr_t` 表项（th_stat / cur_ts 与状态机同步），
  `dma_c` / `cw` 暴露给 burst_sch / dma_ctrl 查询。
- **ts 级互斥锁**：锁声明在 burst 描述里（公共字段 lock_id/unlock_req/lock_req，
  仅 ts 首个 burst st=1 生效），不再从 KOA 旁路输入；lock_req 的 ts 首个 burst
  发射时获取锁，unlock_req 的 ts 完成时释放锁（一段 ts 持锁），线程结束兜底释放；
  同一锁同一时刻只允许一个线程执行（一级发射门控），从根源避免多个线程并发
  操作同一 smc 地址——dma_ctrl 的 loc/free 不再出现跨线程冲突场景。

## 2. 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `MAX_THREADS` | 64 | 线程槽位数 |
| `MAX_TS` | 16 | 每线程最多 ts 数 |
| `MAX_BURST` | 4 | 每个 ts 最多 burst 数 |
| `TS_ID_W` | 6 | ts 编号位宽（0..63，容纳跳转） |
| `CID_W` | 17 | 保序键通道号位宽 |
| `BUF_DEPTH` | 8 | 保序报文缓存深度 |
| `N_LOCK` | 16 | ts 级互斥锁数量（锁 ID 0..15） |

内部位宽派生：`TS_W = $clog2(MAX_TS+1)`（=5，ts 数 1..16）；`th_bs_pc` / `th_off` 为 7bit
（最大 16ts×4burst=64）；`th_sel_ts` 为 4bit（ts 序号 0..15）；`th_sel_idx` 为 3bit
（burst 序号 0..3）。

## 3. 输入输出接口

### 3.1 KOA 输入（有效信息；线程描述由模板池随机）

| 信号 | 位宽 | 说明 |
| --- | --- | --- |
| `ko_vld` | 1 | 有效（不再承载 48B KO 报文） |
| `ko_stream` | 3 | 来源流 0..6（保序键之一） |
| `ko_cid` | 17 | 通道号（保序键之二，1 时隙粒度） |
| `ko_pos` | 3 | 开销位置（保序键之三） |
| `ko_pre_vld` | 4 | 预读入口指示（4 组，随 KO 报文；=1 时该组预读有效） |
| `ko_dma_addr` | 80 | 预读入口 smc 地址（4×20bit） |
| `ko_pre_op` | 4 | 预读入口操作类型（4×1bit：0=loc 1=free） |
| `ko_rdy` | 1 | 反压：`can_accept && !buf_ok && (!有预读 \|\| pre_buf_rdy)`（缓存放行拍反压 KOA；带预读的 KO 额外要求预读缓存空间） |

> **线程描述来源**：THM 建线程时从线程模板池随机选取（`tpl_get($urandom % N_TPL)`，
> 见 `poe_thread_tpl.sv`），不再从 KOA/激励旁路接收完整线程描述；保序缓存只存
> 有效信息（预读 + stream/cid/pos）。

### 3.2 th_sch 一级发射

| 信号 | 位宽 | 说明 |
| --- | --- | --- |
| `ready_mask` | 64 | READY 且未到头的线程位图 |
| `ready_pri` | 192 | **每线程优先级**（3bit×64；同一线程所有 burst 共用，th_sch 按 (pri,tid) 选线程） |
| `ready_burst_ts` | 384 | 发射 burst 所属 **ts 编号**（6bit×64，bs_pc 推导） |
| `ready_curts` | 384 | 当前 **ts 编号**（6bit×64，done 推进） |
| `ready_burst` | 2048 | 发射 burst（bs_pc 索引，32bit×64） |
| `iss_vld0/1` | 1/1 | 一级发射请求 0/1 |
| `iss_tid0/1` | 6/6 | 发射线程 id |

### 3.3 完成反馈（cur_ts 推进）

| 信号 | 位宽 | 说明 |
| --- | --- | --- |
| `cu_done_vld0/1` / `cu_done_tid0/1` | 1/6 ×2 | i/v task 完成（CU0/CU1 桩返回） |
| `dma_done_vld` / `dma_done_tid` | 1/6 | c_task 完成（dma_ctrl 返回） |
| `emit_vld` / `emit_tid` | 1/6 | burst_sch 二级发射确认通知（**占位未用**；bs_pc 推进由 ISSUED 打拍完成，本端口预留） |

### 3.4 pre_read 预读接口（THM → burst_sch）

| 信号 | 位宽 | 说明 |
| --- | --- | --- |
| `pre_buf_rdy` | 1 | 二级发射预读缓存（8 深）有空间；满时反压带预读的 KO 建线程 |
| `pre_vld` | 4 | 预读请求有效（4 组，线程创建同拍输出） |
| `pre_tid` | 24 | 预读线程归属（4×6bit，= 新建线程 tid） |
| `pre_dma_addr` | 80 | 预读 smc 地址（4×20bit，来自 KOA 入口） |
| `pre_op` | 4 | 预读操作类型（4×1bit，来自 KOA 入口） |

> 出口格式与 dma_ctrl 入口（操作指令）一致：`{vld, tid, dma_addr, op_type}`×4。
> 预读请求不生成 c_task burst、不占 C 窗资源、不回 dma_done，由 burst_sch 最高优先级调度。

### 3.5 CSR 暴露 / 线程释放

| 信号 | 位宽 | 说明 |
| --- | --- | --- |
| `csr_dma_c` | 512 | 每线程 dma_c（8bit×64，burst_sch/dma_ctrl 查询） |
| `csr_cw` | 24576 | 每线程 cw（384bit×64） |
| `pre_dma_c` | 8 | pre 独立占位 dma_c（全 1） |
| `pre_cw` | 256 | pre 独立占位 cw（tag 取 KO 首字节） |

> C 窗资源生命周期完全由 c_task 控制（lock/free 成对出现），**线程结束不再输出
> th_rel 兜底归还**；T_DONE 仅用于线程状态回收（槽位复用）与互斥锁释放。

## 4. 内部数据结构

### 4.1 线程表项（每线程一组）

| 信号 | 位宽 | 含义 |
| --- | --- | --- |
| `th_state` | 2 | 状态机：IDLE / READY / ISSUED / DONE |
| `th_head` | 32 | KO 报文 POE_HEAD（`ko_data[383:352]`） |
| `th_ts_n` | 5 | 线程 ts 数（1..16） |
| `th_pri_r` | 3 | 线程 burst 优先级 |
| `th_burst_r` | 32×16×4 | 每 ts 的 burst 模式（[ts][burst]） |
| `th_stream/th_cid/th_pos` | 3/17/3 | 保序键（线程归属） |
| `th_bs_pc` | 7 | 应发射 burst 的全局流水序号（跨 ts 连续，0..63） |
| `th_cur_ts` | 6 | 当前 **ts 编号**（done 推进后跳转） |
| `th_ts_idx` | 4 | 当前在第几个 ts（序号 0..n-1） |
| `th_ts_id_r` | 6×16 | 每 ts 编号（递增可跳转） |
| `th_wait` | 8 | ISSUED 剩余打拍数（branch 1+3+t / 非 branch 1） |
| `th_done_acc` | 3×16 | 每 ts 已完成 burst 数（done 按 tidx 累加） |
| `th_need` | 3×16 | 每 ts 实际执行 burst 数（首个 branch 提前截断） |
| `th_off` | 7×17 | 每 ts 起始累计偏移（off[k] = 前 k 个 ts 总长，组合） |
| `th_sel_ts` | 4 | bs_pc 所属 ts 序号（组合） |
| `th_sel_idx` | 3 | bs_pc 在所属 ts 内的 burst 序号（组合） |

### 4.2 CSR 表项（`csr_t`，见 poe_types_pkg）

`err(8) / ccr(64) / sys_ts(48) / th_id(6) / th_stat(8) / o_mes(8) / cur_ts(8) /
vtsk_c(8) / dma_c(8) / tw(64) / cw(8×48)`。

> **CSR 表项是 THM 内部数据结构**（`csr[MAX_THREADS]`），由 THM 建线程时生成、状态机与
> dma_ctrl 回写维护，不从 KOA 直接输入。KOA 输入侧只有**线程描述旁路**（模拟 I_BUF_A，
> 占位）提供的初始化值。

当前同步字段：建线程时写 `sys_ts / th_id / th_stat / cur_ts / vtsk_c / dma_c / cw`；
`th_stat` 随状态机（READY/ISSUED/DONE/IDLE），`cur_ts` 随 done 推进写 ts 编号；
`err / ccr / o_mes / tw` 为占位 0。

### 4.4 ts 级互斥锁表（`lock_owner[16]`）

| 信号 | 位宽 | 含义 |
| --- | --- | --- |
| `lock_owner[i]` | 6 | 锁 i 的持有线程 tid；`63`=空闲 |

> 锁以 ts 为单位：lock_req 的 ts 首个 burst 发射时获取锁（登记 lock_owner）；
> unlock_req 的 ts 完成时释放锁（该 ts 也在锁保护下执行完）；一段 ts 持锁，
> 中间 ts 无需重复声明。同锁其他线程的 `ready_mask` 置 0（等待），锁释放后恢复
> 可发射。锁声明在 burst 公共字段（lock_id/unlock_req/lock_req，仅 st=1 生效）。

### 4.3 保序报文缓存（`buf_mem[BUF_DEPTH]`）

每条缓存一个完整 KO+线程描述打包（PKG_W = 3012bit），布局（高位→低位）：

```
lock(5) | pre(88) | ko_data(384) | stream(3) | cid(17) | pos(3) | ts_cnt(5) | ts_id(96) |
bs_cnt(48) | pri(3) | burst_seq(2048) | vtsk_c(8) | dma_c(8) | cw(384)
```

缓存带 head/tail/cnt（8 深环形 FIFO），`buf_ok` 表示队头可放行建线程。

## 5. 关键函数与组合逻辑

| 函数/块 | 位置 | 功能 |
| --- | --- | --- |
| `key_active(s,c,p)` | 131 | 遍历线程池，返回是否存在同 key（非 IDLE）活跃线程 |
| `find_idle()` | 138 | 返回第一个 IDLE 槽位，无则 -1 |
| `ko_poe_head(d)` | 144 | 取 KO 报文 POE_HEAD（高 32bit） |
| `th_off` 前缀和 | 229 | off[k+1] = off[k] + th_need[k]（每线程 17 项） |
| `th_sel_ts/th_sel_idx` | 238 | 由 bs_pc 查 off 得 (ts 序号, 段内 burst 序号) |
| ready 生成 | 249 | ready_mask / ready_burst_ts(编号) / ready_curts / ready_burst |
| `branch_cnt` | 264 | 当拍 READY 且当前 burst 为 branch 的线程数（打拍随机上限） |
| `th_rel` 检测 | 217 | 遍历找 T_DONE 线程，输出 th_rel_vld/tid |

## 6. 处理流程（always_ff 逐拍）

### 6.1 建线程（缓存放行优先，否则直接到达）

1. 计算 `t = find_idle()`、`from_buf = buf_ok`；
2. 来源二选一：
   - `buf_ok`：从 `buf_mem[buf_head]` 解包全部线程描述字段（含 ts_id/bs_cnt/burst_seq）；
   - 否则 KO 直接到达且 `can_accept && !key_active`：采样 `th_ts_cnt / th_bs_cnt /
     th_ts_id / th_pri / th_burst_seq / th_vtsk_c_seq / th_dma_c_seq / th_cw_seq`；
3. 初始化线程槽：状态 T_READY、ts_id_r 逐 ts 拷贝、每 ts 计算 `th_need`（默认 bs[k]，
   首个 i/v branch burst 提前截断为 k+1）、`th_bs_pc=0`、`th_cur_ts=ts_id[0]`（约定 0）、
   `th_ts_idx=0`、done/wait 清零；
4. 同步写 CSR（sys_ts 取当前时戳，th_stat=T_READY，cur_ts=0）；
5. 若来自缓存：head+1、cnt-1。

### 6.2 保序入缓存

KO 直接到达但 `key_active`（同 key 线程仍活跃）且缓存未满：整包写入 `buf_mem[buf_tail]`，
tail+1、cnt+1。缓存满时 `can_accept=0` → `ko_rdy=0` 反压，vld 保持。

> **缓存放行拍反压策略**：KO 每拍最多 1 条，THM 每拍只能创建 1 个线程。当缓存队头可放行
> （`buf_ok=1`）时，本拍专用于缓存报文建线程，同时 `ko_rdy` 拉 0（固定 1 拍），让同拍
> 直接到达的 KO 保持 vld、下一拍再处理，避免"缓存放行让位导致新报文丢失"。多个保序
> 阻塞连续解除（`buf_ok` 连续多拍为 1）时反压连续拉长，保证每个缓存报文都有创建机会。

### 6.3 pre_read 预读转发（线程创建同拍，不替代建线程）

带预读的 KO 先正常进入 6.1/6.2 的建线程或入缓存路径；**建线程成立的那一拍**，根据入口
预读指示（`ko_pre_vld` 4 组）把对应的 `dma_addr / op_type` 连同新建线程 tid 输出到二级发射
（`pre_vld / pre_tid / pre_dma_addr / pre_op`，≤4 组）。预读请求不生成 c_task burst、
不占 C 窗资源、不回 dma_done。

> 反压：二级发射预读缓存（8 深）满时 `pre_buf_rdy=0`，`ko_rdy` 对带预读的 KO 拉低，
> 报文 vld 保持、建线程与预读转发一起延后，避免"只建线程不转发预读"。

### 6.4 一级发射（th_sch 选中）

`iss_vld0/1` 且线程 READY 且 `bs_pc < th_off[th_ts_n]`（总长未到头）：

- 取当前 burst `th_burst_r[th_sel_ts][th_sel_idx]`；
- 状态 → ISSUED，CSR.th_stat 同步；
- 打拍数：非 branch 1 拍；i/v branch `4 + $urandom % (branch_cnt+1)`（即 1+3+t）。
- **互斥锁获取**：当前 burst 若为 ts 首个且 lock_req=1，发射拍若锁空闲则登记持锁
  （`lock_owner[lock_id] <= tid`）；同拍两个发射同锁时让 iss0 优先。

> 一级发射门控（ready_mask）：当前 burst 为 ts 首个且 lock_req=1 时，仅当
> `lock_owner[lock_id]==tid || lock_owner[lock_id]==FREE` 可发射；否则
> ready_mask=0 等待，直到持锁线程释放。

### 6.5 ISSUED 打拍推进 bs_pc

ISSUED 且 `th_wait <= 1`：回 READY、CSR.th_stat 同步、`bs_pc+1`；否则 wait-1。
bs_pc 跨 ts 连续递增，因此 burst 队列可包含当前 ts 及更靠后 ts 的 burst
（`ready_burst_ts` 由 bs_pc 经 off 映射为 ts 编号）。

### 6.6 cur_ts 推进（cu0/cu1/dma 三路 done 聚合）

三路完成反馈（线程非 IDLE/DONE）先按 tid 聚合（同 tid 同拍多 done 累加计数），
再逐事件推进：

- `th_done + cnt < th_need[th_ts_idx]`：`th_done + cnt`；
- 累计 ≥ need 时跨 ts 循环推进：`th_ts_idx+1`、`th_cur_ts` 跳转到下一 ts 编号、
  CSR.cur_ts 同步，溢出计数带入下一 ts；末 ts 完成 → 线程 T_DONE，CSR.th_stat 同步。

> 支持同一线程两个 CU 同拍完成（done 合并）以及 done 跨 ts 边界。

> 注意：cur_ts 推进与 bs_pc 推进解耦（bs_pc 打拍推进可能超前）；burst 队列中不应出现
> 当前 ts 以前的 burst（ts 编号比较在 burst_sch 完成）。

### 6.7 线程释放

时序侧将 T_DONE 回 IDLE、CSR.th_stat 同步，槽位可复用；unlock_req 的 ts 完成时
释放锁（done 推进跨过该 ts 时 `lock_owner<=FREE`），线程结束兜底扫描释放本线程
持有的全部锁，等待同锁的线程恢复可发射。C 窗资源不随线程释放（lock/free 由
c_task 成对控制）。

## 7. 占位 / 待细化项

- `emit_vld/emit_tid`（burst_sch 二级发射通知）声明但未使用；
- 200µs 回归：ts 级互斥锁机制下同锁线程互斥执行，loc/free 严格成对，
  共享资源 **alloc=free**（零泄漏，f_cnt=256），SCB 错配 0；
- pre_read 接口已按方案实现：KOA 预读随报文入 SBUF（出队对齐输出）→ THM 建线程同拍
  转发（4 组 {vld,tid,dma_addr,op}）→ burst_sch 8 深预读缓存（最高优先级调度、满反压）；
  dma_ctrl 预读入口接收即吸收（占位，预读执行语义待细化）；
- `vtsk_c` 仅存表，CU 桩未按 tsk_id 判定 i/v 任务是否真实执行；
- branch 等待的随机 `t` 上限 = 当拍 READY 的 branch burst 数（模型化预取访问时延 3+t）；
- CSR 的 `err / ccr / o_mes / tw` 为占位 0；
- 线程描述（ts 数/编号/burst 模式）由 tb_top 旁路随机给定，替代真实 I_BUF_A 读取。

## 8. 相关文件

- 代码：[poe_thm.sv](/C:/Users/92541/Documents/ChatGPT/framer/uvm_poe/rtl/poe_thm.sv)
- 数据类型：[poe_types.sv](/C:/Users/92541/Documents/ChatGPT/framer/uvm_poe/rtl/poe_types.sv)
- 线程描述生成：[tb_top.sv](/C:/Users/92541/Documents/ChatGPT/framer/uvm_poe/tb/tb_top.sv)
- 早期草案：[THM设计方案.md](/C:/Users/92541/Documents/ChatGPT/framer/docs/THM设计方案.md)
