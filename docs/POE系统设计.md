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
| dma_ctrl 简化 | 无 C 窗 loc/free 生命周期：只做 RBA 读/写；同地址互斥由 THM 锁在一级发射保证 |

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
th_sch（一级发射，全局 (pri,tid) 取前 2；q0/q1 仅存 i/v，c_task 解析入独立缓存）
   │  q0/q1 i/v 槽池（8×2）+ c_task 缓存（16）
   ▼
burst_sch（二级发射：q0→EU0 / q1→EU1 各自按优先级选；4 个 DSE 调度器选 c_task）
   ├─ i/v burst → EU0/EU1（各含 4 个 CU 桩，≤2 task 分发给 2 个）→ eu_done
   └─ c_task   → DSE 调度器发射即完成 → dma_done（发射的 c_task 推入 dma_ctrl FIFO）
```

完成反馈（eu_done / dma_done）携带 ts 序号回 THM，驱动 `cur_ts` 推进与锁释放。

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

- 线程 = 若干 ts（1..16，编号可跳转），每个 ts = 若干 burst（1..8）；
- ts0 固定 1 个单 i_task burst；
- 同一配对（同 tag/同锁）的 c_task loc/free 存在 RBA 读→写的数据先后依赖，
  分处**不同 ts**，顺序由 burst_sch 的 `ts == cur_ts` 检查保证；不同锁/
  不同 tag 的 loc/free 相互独立，无先后要求；
- 每 ts 首个 burst（st=1）可声明 ts 级锁操作；
- 线程描述（ts 数/编号/burst 模式/pri/vtsk_c/dma_c/cw）由 THM 从模板池随机选取。

### 3.4 CSR 表项（每线程）

`err(8)/ccr(64)/sys_ts(48)/th_id(6)/th_stat(8)/o_mes(8)/cur_ts(8)/vtsk_c(8)/dma_c(8)/tw(64)/cw(8×48)`。

- `dma_c`：c_task 执行掩码（dma_id 查询）；`cw`：c_task 操作表（tag/op_type/r/o/c_line_num/...）；
- 建线程时由模板初始化；dma_ctrl **只读**（按 dma_id 查 op_type/tag），不回写。

### 3.5 ts 级互斥锁表（16 个）

| 字段 | 位宽 | 说明 |
| --- | --- | --- |
| lock_owner[i] | 6bit | 锁 i 持有线程 tid；63=空闲 |

锁生命周期：lock_req 的 ts 首个 burst 发射时获取；unlock_req 的 ts 完成时释放；
线程结束兜底释放本线程持有的全部锁。

### 3.6 C 窗（dma_ctrl）

| 结构 | 定义 | 说明 |
| --- | --- | --- |
| 每线程独享 | 64×8 固定位置（全局号 = tid×8 + dma_id） | cw 8 项全部映射，纯数据存储，无 loc/free 生命周期 |

C 窗条目 168bit：`tag(20)/c_line(128)/d/o/r/cnt(9)/ind(8)`。

- C 窗地址即 cw 条目索引：cw 条目 k（dma_id=k）固定对应 `c_wnd[tid][k]`；
- dma_ctrl 只读 cw（op_type/tag）执行 RBA 读/写，**不再维护**
  C 窗 o/d/r/cnt/ind，也不回写 cw 的 r/o/c_line_num/start_ts/occ_ts
  （cw 条目 / C 窗条目字段保留定义）。

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

发射拍：lock burst 且锁空闲 → 登记 `lock_owner[lock_id]=tid`；
**同拍互斥**：th_sch 对两个 winner 若同为同锁 id 的 loc burst（st=1 && lock_req），
只发高优先者，次者保持 READY 下拍重选。

### 5.5 cur_ts 推进

- done 携带 ts 序号（tidx），按 tidx 累加对应 ts 完成数；
- 当前 ts 完成数 ≥ need → 跨 ts（ts 编号跳转），末 ts 完成 → T_DONE；
- 跨过 unlock_req 的 ts → 释放锁；T_DONE 兜底释放本线程全部锁。

### 5.6 内部表项

`th_state / th_ts_n / th_pri_r / th_burst_r[ts][burst] / th_stream/cid/pos /
th_bs_pc / th_cur_ts / th_ts_idx / th_ts_id_r / th_wait / th_done_acc[ts] /
th_need[ts] / th_off / th_sel_ts / th_sel_idx`。

## 6. th_sch（一级发射）

- 每拍**全局**按 `(pri, tid)` 取前 2 个 READY 线程发射（无奇偶分组）；
- **队列拆分**：q0/q1（8 深槽池）仅存放 i/v burst（槽项含优先级，供无头阻调度）；
  c_task burst 解析出有效 c_task（c0/c1 + dma_c 判有效、cw 解析 tag/op），
  存入独立 c_task 缓存（16 深槽池），满则反压 c_task 发射；
- 槽池无先后关系：二级发射按优先级选任意有效槽，ack 逐槽清空；
- 完成反馈携带 ts_idx（发射时随 burst 下发）。

## 7. burst_sch（二级发射）

**无头阻**：槽池无 FIFO/先后关系，调度器按优先级选任意有效槽（每拍各 ≤1）。

- **i/v**：q0 → EU0、q1 → EU1 各自独立调度，每拍最多 2 个；
  可发射条件：`ts == 线程当前 cur_ts`（pre 槽跳过）、非 O 窗反压；
- **c_task**：4 个独立 DSE 调度器（DSE 0..3，DSE==tag），各从 c_task 缓存按
  优先级选 `tag==DSE` 且 `ts == cur_ts` 的最高优先项，每拍最多 4 个
  （每 DSE 1 个）；**发射即完成**——done 由二级发射寄存 1 拍直接返回 THM，
  发射的 c_task 推入 dma_ctrl 对应 DSE 的 c_task FIFO，**FIFO 满时反压该 DSE
  发射**（槽保留，下拍重选）；`ts==cur_ts` 保证同配对 loc/free 的先后顺序；
- 无需资源预检/冲突检查：同地址互斥由 THM 锁在一级发射保证。

## 8. CU / EU

- 2 个 EU 桩，每个内含 4 个 CU 桩（task0→CU0、task1→CU1，CU2/CU3 预留）：
  接收 i/v burst，≤2 个 task 分发给 CU 桩执行（LATENCY=1），
  全部完成后聚合回 1 个 `eu_done{vld, tid, ts_idx}`（按 burst 计）；
- 真实 CU 语义（按 tsk_id 查 vtsk_c、按 sub_pc 取子指令）为后续细化项。

## 9. dma_ctrl（c_task FIFO 存储缓冲）

### 9.1 定位

解析已在 th_sch 完成（tid/tidx/dma_id/tag/op），c_task 由 4 个 DSE 调度器
（DSE 0..3，DSE==tag）按优先级选出并发射；**发射即完成**：done 在二级发射
寄存 1 拍后直接返回 THM（无执行延迟/无 RBA/SMC/C 窗数据通路）。

dma_ctrl 保留 **4 个 c_task FIFO**（每 DSE 1 个，默认 8 深，存
`{tid, tidx, dma_id, tag, op}`），作为二级发射与 dma 侧之间的存储缓冲：
发射的 c_task 推入对应 FIFO，**FIFO 满时反压二级发射**（该 DSE 本拍不发射，
c_task 留在缓存下拍重选）；每个 FIFO 每拍出队 1 个（模型化消化，腾出空间）。

同地址互斥由 THM 锁在一级发射保证。

### 9.2 RBA 读写机会（poe_rba）

- 4 个计数（每 DSE 1 个，对应 c_task FIFO），初始各 64 个读写机会；
- dma 每出队 1 个 c_task（发起 1 次读/写请求），对应计数 -1；
- 请求后随机 70~75 拍释放读写机会，计数 +1；
- 计数为 0 时 `rdy=0` 反压：dma_ctrl 停止出队 → FIFO 填满 → 二级发射被反压；
- 参数：`CREDITS=64`、`DELAY_MIN=70`、`DELAY_MAX=75`，`rba_bp/rba_cnt` 可观测。

完成反馈：`dma_done{vld, tid, ts_idx}`（4 路，每 DSE 1 路，发射后 1 拍）。

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
| ts / burst 上限 | 16 / 8（每 ts 最多 8 个 burst） |
| i/v 槽池（q0/q1） | 8 深 ×2（槽项含 pri） |
| c_task 缓存 | 16 深（解析后的单 task：pri/tid/tidx/ts/dma_id/tag/op） |
| dma_ctrl c_task FIFO | 4 个（每 DSE 1 个，8 深；满则反压二级发射） |
| RBA 读写机会 | 4 个计数（初始 64；请求 -1，70~75 拍归还 +1；为 0 反压） |
| 执行单元 | EU ×2（各 4 个 CU 桩）；c_task 无执行单元（二级发射即 done） |
| burst 位宽 | 38bit |
| 锁表 | 16 × 6bit |
| C 窗 | 每线程独享 8 位置（64×8，纯数据存储，无 loc/free 生命周期） |
| cw 条目 / C 窗条目 | 48bit / 168bit |
| KO 有效信息 | 111bit/条（KOA 缓存项） |
| 完成反馈 | {vld, tid(6), ts_idx(4)} |

## 12. 待细化项

- CU 真实语义（vtsk_c 判定 / I_BUF_B 子指令）与 O 窗设计；
- pre_read 预读执行语义（当前 dma_ctrl 接收即吸收）；
- SMC/RBA 总线握手与深度；
- CSR 的 err/ccr/o_mes/tw 语义；
- 线程模板池与锁模板的最终方案（当前 6 个模板，两套模板库待统一）。
