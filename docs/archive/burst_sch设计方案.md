# burst_sch 设计方案（POE 子模块）

> 状态：初步方案草案，供讨论与迭代。

## 1. 模块定位

burst_sch（宏指令调度器）负责**二级发射**：

- 控制 burst 队列中的可发射 burst 进行发射；
- **burst 是 task 的载体**：一个 burst 内包含最多 2 个 task 的信息（i/v_burst 含 i_task/v_task，c_burst 含 c_task）；
- 每拍**同时从两个 burst 队列各取 1 个**可发射 burst，最多发射 **2 个 burst**；
- 按 burst 类型分发：**c_burst** 送给 **dma_ctrl**，**i/v_burst** 送给 **EU**。

## 2. 对外接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| burst 队列 0/1 → burst_sch | 可发射 burst | 每拍各取 1 个（≤2 burst） |
| burst_sch → dma_ctrl | c_burst | 含最多 2 个 c_task |
| burst_sch → EU | i/v_burst | 含最多 2 个 i_task 或 v_task |

## 3. 内部结构

- **队列读取 / 仲裁**：从 2 个 burst 队列读取可发射 burst；
- **类型分发**：按 burst_type 判定，路由到 dma_ctrl 或 EU。

## 4. 发射流程

1. 每拍同时从两个 burst 队列各取 1 个可发射 burst（最多 2 个）；
2. 按 burst_type 分发：c_burst → dma_ctrl；i/v_burst → EU；
3. 目标端按 burst 内容取最多 2 个 task 执行。

## 5. 待确认项

- 队列与 dma_ctrl / EU 之间的背压与握手；
- task 拆分发生在 burst_sch 还是目标端（dma_ctrl / EU）。

## 6. 相关图示

- [burst_sch 设计](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/burst-sch-design-standalone.html)
