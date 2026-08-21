# POE 系统设计（当前版）

> 本文档基于 `uvm_poe` 当前实现（KOA + POE THM/th_sch/burst_sch/CU/dma_ctrl 行为模型）
> 整理，作为全链路完整设计说明。历史文档已归档至 `docs/archive/`。
>
> 核心简化：链路不再承载 48B KO 报文，只传递有效信息（优先级/流/通道/位置/预读）；
> 线程描述由 THM 从线程模板池随机选取；同一 smc 地址的并发冲突由 THM **ts 级互斥锁**
> 从根源避免，dma_ctrl 不再需要扫描/预存/转交等冲突管理。

## 1. 概述与设计目标

POE（Packet Overhead Engine）核处理 fgOTN 业务数据流中的开销指令（KO 指令）。验证平台
以 UVM 搭建 KOA 输入调度 + POE 指令流水（线程化执行模型），用于验证：

- KOA 多业务源（fgOTN OH / X2X APS/ALM / 串口）的优先级调度与不丢报文；
- THM 的保序、线程生命周期、ts 级互斥锁与 CSR 维护；
- th_sch / burst_sch 两级发射调度；
- CU（计算）与 dma_ctrl（DMA / C 窗缓存）执行模型。

**关键设计决策**

| 决策 | 说明 |
| --- | --- |
| 无 KO 数据 | 激励/KOA/THM 均不承载 48B 报文，只传有效信息（pri/stream/cid/pos/预读） |
| 线程模板池 | THM 建线程时从模板池随机取线程描述（ts 结构/burst 模式/CSR 初值） |
| ts 级互斥锁 | burst 描述声明 lock/unlock（仅 ts 首个 burst 生效），同锁线程互斥执行 |
| dma_ctrl 简化 | 无 C 窗扫描/指令预存/转交，loc 直接申请、free 直接释放 |

## 2. 系统架构与数据流

```text
UVM 激励（fgOTN OH / X2X / 串口）
   │  vld/pri/cid/pos/pre_read（无 48B KO）
   ▼
KOA（5×SBUF × 8 优先级段，SP+RR 调度）
   │  out_vld/pri/src/stream/cid/pos/pre_read
   ▼
THM（线程池 64，保序 8 深缓存，模板池取线程描述）
   │  ready_mask/ready_burst（38bit）
   ▼
th_sch（一级发射，tid 奇偶队列亲和，每拍 ≤2）
   │  q0/q1 burst 队列（8 深）
   ▼
burst_sch（二级发射，ts==cur_ts 等条件）
   ├─ i/v_task → CU0/CU1（计算桩）→ cu_done
   └─ c_task  → dma_ctrl（loc/free + C 窗 + RBA/SMC）→ dma_done
```

完成反馈（cu_done / dma_done）携带 ts 序号回 THM，驱动 `cur_ts` 推进与锁释放。

## 3. 数据结构

### 3.1 KO 有效信息（替代 48B 报文）

| 字段 | 位宽 | 含义 |
| --- | --- | --- |
| pri | 3bit | 调度优先级（0 最高），决定 KOA 段号 |
| stream | 3bit | 来源流 0..6（保序键之一） |
| cid | 17bit | 通道号（1 时隙粒度，保序键之二） |
| pos | 3bit | 开销位置（保序键之三） |
| pre_read | 88bit | 预读指示（4×{vld, dma_addr(20), op}） |

### 3.2 burst（38bit）

一种结构、两种类型（`burst_type` 区分 0=i/v、1=c_task），字段按类型复用；锁字段为
公共字段，**仅 ts 首个 burst（st=1）生效**：

| 字段 | 位宽 | 说明 |
| --- | --- | --- |
| lock_id | 4bit | 锁 ID（0..15） |
| unlock_req / lock_req | 1bit ×2 | ts 级解锁 / 加锁请求 |
| st | 1bit | 是否为所属 ts 首个 burst |
| tr | 1bit | 是否涉及 O 窗操作 |
| ts_len / rev | 3bit / 4bit | i/v 所属 ts burst 数 / c 保留 |
| branch | 1bit | 仅 i/v：是否可能 ts 跳转 |
| burst_type / vld_cu | 1bit ×2 | 类型 / 需执行 task 数 |
| tsk_id / dma_id | 3bit ×2 | 任务 id（查 CSR.vtsk_c / dma_c） |
| c0 / c1 | 1bit ×2 | 任务有效标志 |
| sub_pc / occ_ts | 8bit ×2 | i/v 指令指针 / c 占据 ts 数 |

### 3.3 线程结构

- 线程 = 若干 ts（1..16，编号可跳转），每个 ts = 若干 burst（1..4）；
- ts0 固定 1 个单 i_task burst；
- 每 ts 首个 burst（st=1）可声明 ts 级锁操作；
- 线程描述（ts 数/编号/burst 模式/pri/vtsk_c/dma_c/cw）由 THM 从模板池随机选取。

### 3.4 CSR 表项（每线程）

`err(8)/ccr(64)/sys_ts(48)/th_id(6)/th_stat(8)/o_mes(8)/cur_ts(8)/vtsk_c(8)/dma_c(8)/tw(64)/cw(8×48)`。

- `dma_c`：c_task 执行掩码（dma_id 查询）；`cw`：c_task 操作表（tag/op_type/r/o/c_line_num/...）；
- 建线程时同步生成，状态机与 dma_ctrl 回写维护。

### 3.5 ts 级互斥锁表（16 个）

| 字段 | 位宽 | 说明 |
| --- | --- | --- |
| lock_owner[i] | 6bit | 锁 i 持有线程 tid；63=空闲 |

锁生命周期：lock_req 的 ts 首个 burst 发射时获取；unlock_req 的 ts 完成时释放；
线程结束兜底释放本线程持有的全部锁。

### 3.6 C 窗（dma_ctrl）

| 结构 | 定义 | 说明 |
| --- | --- | --- |
| 每线程独享 | 64×8 固定位置（全局号 = tid×8 + dma_id） | cw 8 项全部映射，无共享池 / FIFO / 占用计数 |

C 窗条目 168bit：`tag(20)/c_line(128)/d/o/r/cnt(9)/ind(8)`。

- C 窗地址即 cw 条目索引：cw 条目 k（dma_id=k）固定对应 `c_wnd[tid][k]`；
- `cw.c_line_num` / C 窗条目 `ind` 只存线程内位置索引（= dma_id，0..7）；
  全局位置 `tid×8+dma_id` 最大 511，需 9bit，由 dma_ctrl 内部计算
  （禁止用 8bit 存全局号，tid≥32 时回绕会释放到错误线程）。

## 4. KOA（输入调度）

### 4.1 结构

- 5 个独立 SBUF：EXT / INS / ALM / UART_EXT / UART_INS（深度 2560）；
- 每个 SBUF 地址按优先级拆 8 段（pri 0..7，0 最高，每段 320 深）；
- 缓存项只存有效信息：`{pre_read(88), stream(3), cid(17), pos(3)}`（111bit）。

### 4.2 写入（同段合并向量写）

- 业务源 vld 直写对应段（按 pri 分段），同段同拍可合并写多条（上限 16）；
- 写入顺序：APS 类（编小优先）→ OH 类（编小优先）→ UART（APS 固定靠前）；
- 段空间不足时后排候选让位（rdy=0、vld 保持，不丢报文）。

### 4.3 读取（SP + RR）

- 每拍 1 条：先按优先级选组（SP，组号 0..7 最高优先）；
- 组内 5 个 SBUF 按 rr_ptr（每拍推进）轮询，取最先非空段出队；
- 输出 `out_vld/out_pri/out_src/out_stream/out_cid/out_pos/out_pre_*`（寄存一拍）。

## 5. THM（线程管理器）

### 5.1 职责

接收 KOA 有效信息，按 `(stream,cid,pos)` 保序建线程，为 th_sch 提供可发射 burst，
并根据 CU/dma_ctrl 完成反馈推进 `cur_ts` 与释放 ts 级锁。

### 5.2 保序

- 新报文若同 key 已有活跃线程 → 入 8 深缓存等待（FIFO 放行）；
- 缓存满 / 线程池满 → `ko_rdy=0` 反压；
- 缓存放行拍专用于缓存建线程，并对 KOA 拉 1 拍反压。

### 5.3 建线程（模板池）

- 线程描述从模板池随机选取：`tpl = tpl_get($urandom % N_TPL)`；
- 解出 ts_cnt / 每 ts burst 数 / ts 编号 / pri / burst 模式 / vtsk_c / dma_c / cw；
- 同步生成 CSR 表项（th_stat=T_READY、cur_ts=ts_id[0] 等）。

### 5.4 一级发射门控（ready_mask）

线程 READY 且满足：

- `bs_pc < th_off[ts_n]`（总流水未到头）；
- `th_sel_ts ∈ [th_ts_idx, th_ts_idx+1]`（只提前 ≤1 个 ts）；
- ts 级锁：当前 burst 若 `st=1 && lock_req=1`，仅当锁空闲或本线程持有时可发射。

发射拍：lock burst 且锁空闲 → 登记 `lock_owner[lock_id]=tid`。

### 5.5 cur_ts 推进

- done 携带 ts 序号（tidx），按 tidx 累加对应 ts 完成数；
- 当前 ts 完成数 ≥ need → 跨 ts（ts 编号跳转），末 ts 完成 → T_DONE；
- 跨过 unlock_req 的 ts → 释放锁；T_DONE 兜底释放本线程全部锁。

### 5.6 内部表项

`th_state / th_ts_n / th_pri_r / th_burst_r[ts][burst] / th_stream/cid/pos /
th_bs_pc / th_cur_ts / th_ts_idx / th_ts_id_r / th_wait / th_done_acc[ts] /
th_need[ts] / th_off / th_sel_ts / th_sel_idx`。

## 6. th_sch（一级发射）

- 每拍按 `(pri, tid)` 选线程发射 ≤2 个；
- **队列亲和**：tid 偶数 → q0、奇数 → q1（同线程 burst 保序）；
- 队列项：`{pre, burst(38), burst_ts(6), ts_idx(4), th_id(6)}`（55bit，8 深 ×2）；
- 完成反馈携带 ts_idx（发射时随 burst 下发）。

## 7. burst_sch（二级发射）

从 q0/q1 各自独立判断队头可发射，每拍 ≤2：

- 公共条件：`ts == 线程当前 cur_ts`（pre 插队跳过）、非 O 窗反压；
- c_task 附加：无需资源预检——C 窗每线程独享 8 个固定位置，loc/free 由 ts 级
  互斥锁保证成对与互斥，只要满足公共条件即放行；
- 路由：i/v → CU0/CU1；c_task → dma_ctrl（dma 单路，双 c_task 同拍只发 q0）。

## 8. CU / EU

- 双 CU 桩（LATENCY=1）：接收 i/v burst，延迟后回 `cu_done{vld, tid, ts_idx}`；
- 真实 CU 语义（按 tsk_id 查 vtsk_c、按 sub_pc 取子指令）为后续细化项。

## 9. dma_ctrl（c_task / C 窗）

### 9.1 定位

执行 c_task（loc/free），维护 C 窗缓存（每线程独享 8 位置）与 SMC/RBA 模型。
互斥锁保证同一地址无并发，因此无扫描查重/指令预存/转交。

### 9.2 loc（0）

- 写固定位置（全局号 = tid×8 + dma_id）的 C 窗条目（o=1、ind=dma_id）；
- 更新 CSR.cw（o=1、c_line_num=dma_id、start_ts、occ_ts）；
- RBA 读 SMC 回填 c_line 并置 r=1。

### 9.3 free（1）

- RBA 把 C 窗 c_line 写回 SMC[tag]；
- 释放：按（cur_tid，tag 匹配出的 loc 条目索引）以 9bit 计算全局位置，清该位置 o
  （不依赖 cw.c_line_num 的存储值，避免 tid≥32 时 8bit 回绕）；
- 更新 CSR.cw（o=0）。

### 9.4 状态机

```text
IDLE → LOAD（拆任务）
  loc  → MISS(申请+写C窗+更新cw) → RBA_RD → RBA_RD_DONE(回填) → NEXT
  free → FREE_RBA(写SMC) → FREE_REL(释放) → NEXT
→ DONE（回 dma_done{tid, ts_idx}）→ IDLE
```

## 10. UVM 平台

- 事务 `koa_item`：有效信息（pri/stream/plane/cid/pos/sbuf），无 KO 报文；
- 激励 seq：fgOTN OH（4 平面）、X2X APS/ALM（8 平面）、串口（60Mpps），
  pri/cid/pos 按时隙表与速率模型产生；
- agent：driver 驱动有效信息接口，monitor 采样输入/输出；
- scoreboard：8 组优先级参考模型（SP+RR），比对输出顺序与组号；
- tb_top：内联 POE 链路（THM/th_sch/burst_sch/CU/dma_ctrl）+ 线程模板池取描述。

## 11. 参数与位宽总表

| 项 | 值 |
| --- | --- |
| 线程池 / 保序缓存 | 64 / 8 深 |
| ts / burst 上限 | 16 / 4 |
| burst 位宽 | 38bit |
| 锁表 | 16 × 6bit |
| C 窗 | 每线程独享 8 位置（64×8，全局号 tid×8+dma_id，9bit） |
| cw 条目 / C 窗条目 | 48bit / 168bit |
| KO 有效信息 | 111bit/条（KOA 缓存项） |
| 完成反馈 | {vld, tid(6), ts_idx(4)} |

## 12. 待细化项

- CU 真实语义（vtsk_c 判定 / I_BUF_B 子指令）与 O 窗设计；
- pre_read 预读执行语义（当前 dma_ctrl 接收即吸收）；
- SMC/RBA 总线握手与深度；
- CSR 的 err/ccr/o_mes/tw 语义；
- 线程模板池与锁模板的最终方案（当前 6 个模板，两套模板库待统一）。
