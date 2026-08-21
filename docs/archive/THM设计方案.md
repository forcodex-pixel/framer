# THM 线程管理器设计方案（POE 子模块）

> 状态：初步方案草案，供讨论与迭代。

## 1. 模块定位

THM（线程管理器）是 POE 内承接 KO 指令流与线程执行的枢纽：

- 接收 KOIU 下发的 48B KO 报文，按报文内容创建并管理线程；
- 维护最多 **32 个并发线程**，线程槽位全部锁定时向前级 KOA 发起反压（i_rdy）；
- 为每个线程管理 Burst（宏指令）的加载、待发射与补充预取，与 IFU、I_BUF_A、th_sch、CU 协同。

## 2. 对外接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| KOIU → THM | 48B KO 报文 | SA / DA / PRI / POE_HEAD / IO_META |
| THM → KOIU | i_rdy（反压） | 单 bit；32 个线程全部锁定时置 0 |
| THM → IFU | 加载请求 | POE_HEAD / 下一地址（默认连续地址） |
| IFU → THM | 已加载 Burst | 宏指令数据 |
| THM → th_sch | 待发射 Burst | 线程 ID + Burst |
| th_sch → THM | 发射完成 | 触发补位预取 |
| CU → THM | 线程释放通知 | CU 执行完 burst 中结束线程的宏指令后通知 |

## 3. KO 报文（6×8B = 48B，共 4 种模板）

KO 报文为 6 行 × 8B 的 48B 结构。

**行 1（所有模板相同的公共表头）**：DA | DP | SA | SP | TYPE | PRI + SN | B/E/LBO(CHN)，其中 PRI 为 3bit、SN 为 13bit（合计 2B），其余字段各 1B。

**行 2**：POE_HEAD 4B + MEM_HEAD 4B（KO 控制报文例外：POE_HEAD 4B + 控制头 4B）。

**行 3–6**（按模板）：

| 模板 | 行 3 | 行 4 | 行 5 | 行 6 |
| --- | --- | --- | --- | --- |
| ① KO 开销报文 | 开销头 8B | 开销头 4B + META 4B | META 8B | META 8B |
| ② KO IO 报文 | META 8B | META 8B | META 8B | META 8B |
| ③ KO 控制报文 | 控制头 8B | RES 8B | RES 8B | RES 8B |
| ④ KO DMA 报文 | DMA META 8B | DMA META 8B | RES 8B | RES 8B |

> 开销头 / META 内的子字段（如 SA / DA / PRI / IO_META 等）位宽后续细化；Burst 大小暂不定义。

## 4. 线程表（32 项）

每项包含：

- 线程 ID / 占用标志；
- 生命周期状态；
- Burst 地址指针（当前 / 下一地址）；
- 待发射 Burst（容量 1，由 Burst 预取控制写入）；
- 完成标志等。

## 5. 线程生命周期

IDLE → LOCKED（建线程）→ WAIT_BURST（等 IFU 加载）→ READY（可发射）→ ISSUED（发射中）→ 预取下一 Burst / COMPLETE → IDLE。

- **线程结束条件**：burst 中包含结束线程的宏指令，CU 执行完后通知 THM 释放线程表项；
- **反压**：32 个线程全部锁定时，i_rdy 置 0 向 KOIU 反压。

## 6. Burst 加载与双缓冲预取

1. 收到 KO 报文 → 锁定线程 → 按 POE_HEAD 通知 IFU 从 I_BUF_A 加载 Burst；
2. Burst 就绪后由预取控制写入该线程表项（容量 1 Burst）；
3. th_sch 发射当前 Burst 后，预取控制将 IFU 已加载的下一 Burst 写入表项，并通知 IFU 加载下一地址；
4. 下一地址默认连续；CU 跳转控制时按跳转结果取址。

## 7. 暂不细化项

- KO 报文各字段位宽；
- Burst 大小；
- i_task / v_task / c_task 三种任务的差异；
- PRI 的使用。

## 8. 相关图示

- [THM 结构与接口](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/thm-architecture-standalone.html)
- [THM 线程生命周期](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/thm-lifecycle-standalone.html)
- [POE 总体结构（上下文）](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/poe-design-standalone.html)
