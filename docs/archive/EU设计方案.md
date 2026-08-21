# EU 设计方案（POE 子模块）

> 状态：初步方案草案，供讨论与迭代。

## 1. 模块定位

EU（计算单元）负责执行 i/v_burst 中的任务：

- 包含 **2 个 CU** 计算单元；
- 收到 burst 后，将 burst 中最多 **2 个 task** 分发给 CU；
- CU 根据 burst 中的 SUB_PC0 / SUB_PC1 从 I_BUF_B 取对应 task 的子指令并执行。

## 2. 对外接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| burst_sch → EU | i/v_burst | 含最多 2 个 i_task 或 v_task |
| CU → I_BUF_B | 读子指令 | 按 SUB_PC0 / SUB_PC1 |
| I_BUF_B → CU | 子指令 | mux + inst_b×n + demux（n≤5） |
| CU ↔ RF（R窗） | 读写 | mux 读地址 / demux 写地址 |
| EU → tr | 计算结果 | 供跳转 / 状态更新 |
| tr → ts_ctrl | 状态更新 | 根据计算结果更新线程状态 |
| tr → O窗 | 结果写回 | 可选，计算结果写至 O窗 |

## 3. 内部结构

- **task 分发**：将 burst 中的 2 个 task 分发给 CU0 / CU1；
- **CU0 / CU1**：各执行一个 task 的子指令序列。

## 4. 任务执行流程

1. EU 收到 i/v_burst，将其中的 2 个 task 分发给 CU0 / CU1；
2. CU 按 SUB_PC0（task0）/ SUB_PC1（task1）从 I_BUF_B 查找对应子指令；
3. 子指令结构为 **mux + inst_b×n + demux**（n ≤ 5）：
   - **mux**：指向 RF（R窗）的读地址；
   - **demux**：指向 RF 的写地址；
   - **inst_b**：具体的计算操作指令；
4. CU 执行 inst_b，经 mux 读 RF、经 demux 写 RF。

**结果处理**：EU 的计算结果送 tr；tr 根据计算结果通知 ts_ctrl 更新线程状态，也有可能将计算结果写至 O窗。

## 5. 待确认项

- task 与 CU 的对应方式（task0 → CU0？按序分配？）；
- 2 个 CU 并发访问 I_BUF_B 的仲裁；
- RF 端口数量与读写冲突；
- inst_b 数量上限 n（≤5）的实际取值。

## 6. 相关图示

- [EU 设计](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/eu-design-standalone.html)
