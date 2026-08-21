# th_sch 设计方案（POE 子模块）

> 状态：初步方案草案，供讨论与迭代。

## 1. 模块定位

th_sch（线程调度器）负责**一级发射**：

- 每拍从 THM 的线程待发射 burst 中，同时选择 **2 个可发射 burst** 进行发射；
- 发射后的 burst 分别存入 **2 个 burst 队列**，每个队列深度为 **8**；
- 队列输出给 burst_sch（宏指令调度器）进行二级发射。

## 2. 对外接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| THM → th_sch | 线程待发射 burst | 可发射集合 |
| th_sch → burst 队列 0/1 | 发射的 burst | 每拍最多 2 个 |
| burst 队列 0/1 → burst_sch | burst | 队列深度 8 |

## 3. 一级发射流程

1. 每拍从可发射 burst 集合中选择 2 个；
2. 发射的 2 个 burst 分别写入 burst 队列 0 与 burst 队列 1；
3. 队列按序输出给 burst_sch，进入二级发射。

## 4. 待确认项

- 每拍选 2 个 burst 的选择策略（轮询 / 其他），PRI 暂不考虑；
- 可发射条件是否由 ts_ctrl 统一判定；
- 队列满 / 空时的背压与握手（与 burst_sch 对接细节）。

## 5. 相关图示

- [th_sch 设计](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/th-sch-design-standalone.html)
