# IFU 设计方案（POE 子模块）

> 状态：初步方案草案，供讨论与迭代。

## 1. 模块定位

IFU（指令加载单元）是 burst 预加载模块：

- 负责从 I_BUF_A 读出预加载的 burst，并缓存在寄存器中；
- 每个线程拥有 **2 深度** 的 burst 存储空间；
- 响应 THM 的 burst 请求，支持带 TS 跳转与不带跳转两种处理路径。

## 2. 对外接口

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| THM → IFU | burst 请求 | TS / 跳转标志 / 线程 ID |
| IFU → THM | burst 返回 | 已预加载的 burst |
| IFU → I_BUF_A | 读请求 | 目标地址 |
| I_BUF_A → IFU | burst 数据 | 宏指令内容 |

## 3. 内部结构

- **请求解析**：接收 THM 的 burst 请求，识别 TS、跳转标志与线程 ID；
- **Burst 寄存器组**：每线程 2 深度，保存已预加载的 burst；
- **地址生成**：无跳转时按顺序取下一地址；有跳转时按新跳转 TS 对应地址；
- **读请求控制**：向 I_BUF_A 发起读请求并接收数据。

## 4. 请求处理流程

- **带 TS 跳转**：按新跳转 TS 对应地址加载 burst → 返回给 THM → 随后预加载新位置的 burst；
- **不带跳转**：返回该线程已预加载的 burst → 再按顺序加载下一个 burst。

## 5. Burst 数据结构

Burst 分为两种：**i/v_burst** 与 **c_burst**，均为 18 bit。

| 类型 | 字段 | 位宽 |
| --- | --- | --- |
| i/v_burst | st / tr / branch / burst_type / vld_cu / c0 / c1 / sub_pc0 / sub_pc1 | 各 1 bit |
| i/v_burst | ts_len | 3 bit |
| i/v_burst | tsk_id0 | 3 bit |
| i/v_burst | tsk_id1 | 3 bit |
| c_burst | st / tr / burst_type / vld_cu / c0 / c1 / occ_ts0 / occ_ts1 | 各 1 bit |
| c_burst | rev | 4 bit |
| c_burst | dma_id0 | 3 bit |
| c_burst | dma_id1 | 3 bit |

> 字段位序：st 为最低位（bit 0），其余按上述顺序依次排列。

## 6. 相关图示

- [IFU 设计](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/ifu-design-standalone.html)
- [THM 架构（含 KO 报文）](C:/Users/92541/.codex/visualizations/2026/08/06/019fd49a-1e82-7920-a852-d56510961252/thm-architecture-standalone.html)
