# dma_ctrl 设计方案与实现详解（POE 子模块）

> 本文档以当前实现为准（`uvm_poe/rtl/poe_dma_ctrl.sv`），对应《数据结构位宽总表.md》
> §5（C 窗）、§7（操作指令接口）、§8（SMC/RBA）。
>
> **简化说明**：THM 线程级互斥锁（16 个独立锁）保证同一 smc 地址同一时刻只有一个
> 线程执行，因此 dma_ctrl 不再需要 C 窗扫描查重、指令预存、转交等冲突管理——
> loc 直接申请资源、free 直接释放，状态机大幅精简。

## 1. 模块定位

dma_ctrl 负责执行 c_burst 中的 DMA 类任务（c_task）：

- 接收 burst_sch 二级发射的 c_task burst（≤2 个 c_task/拍）；
- 按 c0/c1 + CSR.dma_c 判定实际执行的任务，按 dma_id 查 CSR.cw 得到操作类型
  （**loc** 锁定 / **free** 释放）与 smc 地址（tag）；
- C 窗分独享/共享两类（cw 8 项固定映射：dma_id 0..3 → 独享、4..7 → 共享）：
  独享 = 64×4 固定位置（tid×4+k），申请/释放不经过 FIFO；共享 = 256 项，
  由资源管理 FIFO 分配/归还（单线程共享占用上限 4）；
- **loc**：直接申请 C 窗资源（无查重/命中/预存，互斥锁保证无冲突）、写 C 窗条目、
  更新 CSR.cw、经 RBA 读 SMC 数据回填 c_line；
- **free**：同拍同地址去重 → 经 RBA 把 c_line 写回 SMC → cw.o=0 释放资源；
- 完成一个 burst（1~2 个 c_task）后回一次 `dma_done`（THM 据此推进 cur_ts）；
- 资源生命周期完全由 c_task 控制（lock/free 成对出现），**线程结束不兜底归还**；
  dma_done 仅表示 burst 执行完成（THM cur_ts 推进用），不归还资源。

## 2. 对外接口

| 方向 | 信号 | 位宽 | 说明 |
| --- | --- | --- | --- |
| burst_sch → | `emit_dma_vld` | 1 | c_task burst 发射请求（每拍 ≤1，当前单路） |
| | `emit_dma_tid` | 6 | 线程 id |
| | `emit_dma_tidx` | 4 | 完成 burst 的 ts 序号（透传回 THM） |
| | `emit_dma_burst` | 32 | c_burst（c0/c1+dma_id0/1+occ_ts0/1） |
| | `emit_dma_pre` | 1 | pre_read 插队 burst（不占资源/不回 done） |
| ← burst_sch | `dma_ack` | 1 | 空闲可收（内部无任务在执行） |
| THM → | `csr_dma_c` | 512 | 每线程 dma_c（8bit×64） |
| | `csr_cw` | 24576 | 每线程 cw（384bit×64，只读视图） |
| | `thread_curts` | 384 | 每线程当前 ts 编号（写 cw.start_ts 用） |
| → THM | `cw_upd_vld` | 1 | CSR.cw 条目更新请求（loc 申请/free 释放） |
| | `cw_upd_tid` | 6 | 线程 id |
| | `cw_upd_ind` | 3 | cw 条目索引（dma_id） |
| | `cw_upd_data` | 48 | 新 cw 条目（cw_entry_t） |
| → burst_sch | `cw_fifo_cnt` | 10 | C 窗**共享**资源池空闲数（发射条件） |
| | `th_res_n` | 192 | 每线程**共享**占用数（3bit×64，上限 4） |
| → THM | `dma_done_vld`/`dma_done_tid`/`dma_done_tidx` | 1/6/4 | burst 完成（cur_ts 推进） |

> 演进说明：方案中 burst_sch 侧拆出 4 路操作指令（vld+th_id+op_type+smc_addr，28bit/路）。
> 当前实现保持 burst 粒度接口，在 dma_ctrl 内部完成等价拆分（≤2 task/拍），
> 待 burst_sch 双发改造时再前移到接口。

## 3. 内部数据结构

### 3.1 操作指令（内部，28bit/路，≤2 路/拍）

| 字段 | 位宽 | 含义 |
| --- | --- | --- |
| vld | 1 | 有效（c 有效且 dma_c 置位） |
| th_id | 6 | 线程 id |
| op_type | 1 | loc=0 / free=1（来自 cw[dma_id].op_type） |
| smc_addr | 20 | tag（来自 cw[dma_id].tag） |

### 3.2 C 窗（独享 `c_wnd_excl[64][4]` + 共享 `c_wnd_shr[256]`，条目 `c_wnd_entry_t` 168bit）

`tag(20) / c_line(128) / d(1) / o(1) / r(1) / cnt(9) / ind(8)`。

> 独享位置 = tid×4+k（k=0..3，cw dma_id 0..3 固定映射），申请/释放不经过 FIFO；
> 共享资源号 = FIFO 分配（0..255，cw dma_id 4..7 固定映射），`th_res_n_r` 只统计共享占用。

### 3.3 CSR.cw 条目（`cw_entry_t` 48bit）

`tag(20) / op_type(1) / r(1) / o(1) / c_line_num(8) / start_ts(8) / occ_ts(8) / rsv(1)`。

### 3.4 资源池 / SMC

- `f_mem[256]`：**共享**资源号 FIFO（复位入队 0..255），`f_head/f_tail/f_cnt`；
- `th_res_n_r[64]`：每线程**共享**占用数（loc 才 +1，free 归还 -1，上限 4）；
- `smc_mem[256][128]`：SMC 缓存模型（tag 低 8bit 索引），RBA 读/写各 1 拍延迟。

## 4. 处理流程

### 4.1 接收与拆任务

`emit_dma_vld && !busy && !pre` 时锁存 burst/tid，拆出 ≤2 个有效任务：

- task0：`c0 && dma_c[dma_id0]`；task1：`vld_cu && c1 && dma_c[dma_id1]`；
- 每个任务按 `dma_id` 取 `cw[dma_id]`（tag / op_type），形成内部操作指令；
- **同拍去重**：同拍两个任务若均为 free 且 smc 地址相同，只执行 task0（端口小者优先）。

### 4.2 loc（锁定，0）

1. **申请资源**：独享（dma_id<4）用固定位置 `tid×4+dma_id[1:0]`（不占 FIFO）；
   共享（dma_id≥4）`f_mem[f_head]` 出队（`th_res_n_r+1`，上限 4）；
   （互斥锁保证无冲突，不做查重/命中判断，直接申请）
2. 写 C 窗条目：`{tag, c_line=0, d=0, o=1, r=0, cnt=0, ind=资源号}`；
3. 更新 CSR.cw：`{tag, op_type, r=0, o=1, c_line_num=资源号,
   start_ts=线程当前 ts 编号, occ_ts=burst.occ_ts, rsv=0}`（`cw_upd` 请求）；
4. **RBA 读** SMC[tag]（1 拍延迟），数据回填 C 窗 `c_line` 并置 `r=1`，
   同时更新 cw.r=1。

### 4.3 free（释放，1）

1. 取 `cw[dma_id]` 的 `c_line_num`（独享 = tid×4+k / 共享 = FIFO 资源号）→ 对应条目；
2. **RBA 写**：把 C 窗 `c_line` 写入 `smc_mem[tag 低 8bit]`（1 拍延迟）；
3. **释放**：`cw.o=0`（`cw_upd` 请求），C 窗条目 `o=0`；独享不入 FIFO，共享资源号
   入 FIFO 且 `th_res_n_r-1`。

### 4.4 完成 / 归还

- 一个 burst 的全部有效任务执行完 → `dma_done_vld=1`（1 拍），busy 清；
- 每次 loc 申请的资源由对应 free 归还（共享资源 FIFO 入队、th_res_n_r-1）；
- pre 插队 burst：不占资源、不执行 loc/free、不回 done（占位，预读语义待细化）。

## 5. 状态机（单任务串行流水）

```
IDLE --接收--> LOAD（锁存/拆任务）
  task0: loc → MISS(申请+写C窗+更新cw+RBA读回填)
         free → RBA_W(写SMC) → FREE(释放)
  task1: 同上（task0 完成后进入）
  全部完成 → DONE（回 dma_done 1 拍）→ IDLE
```

`dma_ack = !busy`；busy 从接收拍到完成保持。

## 6. 与现有资源管理的衔接

- burst_sch 的发射条件仍按"每个有效 c_task 需 1 个资源"保守预检查
  （`cw_fifo_cnt ≥ N_loc_shared && 线程共享占用 + N_loc_shared ≤ 4`）；独享任务
  固定位置不占共享池，free 任务无条件放行（lock/free 成对，不因资源条件阻塞）；
- 资源收支按任务粒度：loc 申请（独享不计 / 共享 +1）、free 释放归还，
  线程结束不兜底（资源生命周期完全由 c_task 控制）。

## 7. 占位 / 待细化项

- SMC/RBA 总线握手与深度（模型：256×128bit，tag 低 8bit 索引，1 拍延迟）；
- C 窗条目 `cnt` 老化计数（超上限强制释放）未实现；
- pre 插队 burst 的预读语义（pre_mes 4 组接口）未实现；
- 操作指令 4 路接口待 burst_sch 双发改造时前移；
- CU 改写 c_line 置 `d=1` 的回写路径未接（dma_ctrl 只读 C 窗）。

## 8. 相关文件

- 代码：[poe_dma_ctrl.sv](/C:/Users/92541/Documents/ChatGPT/framer/uvm_poe/rtl/poe_dma_ctrl.sv)
- 数据类型：[poe_types.sv](/C:/Users/92541/Documents/ChatGPT/framer/uvm_poe/rtl/poe_types.sv)
- 位宽总表：[数据结构位宽总表.md](/C:/Users/92541/Documents/ChatGPT/framer/docs/数据结构位宽总表.md)
