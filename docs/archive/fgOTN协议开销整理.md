# fgOTN 协议开销整理

> 整理对象：fgOTN（fine grain OTN，细粒度光传送网）协议中定义的开销（Overhead）内容。
>
> 标准依据：
> - ITU-T G.709.20 (04/2024)《Overview of fine grain OTN》及其 Amd.1 (05/2025)——fgOTN 功能总览、开销需求清单
> - ITU-T G.709/Y.1331 (2020) Amd.3 / Amd.4 (07/2025)——fgODUflex 帧结构与开销的具体定义
>   - Annex M：fgODUflex 路径层（帧格式、开销、维护信号、业务映射）
>   - Annex N：fgODUflex → fgODTU → OPU 细粒度时隙（fgTS）的映射与复用开销
>   - Annex O：fgODUflex 无损伤带宽调整（resizing）控制开销
>
> 整理日期：2026-08-05

---

## 1. 背景

fgOTN 是 ITU-T 为承载 sub-1Gbit/s 业务（如 E1、STM-1/4、VC-n、10M 粒度的以太网）而定义的细粒度 OTN 路径层，用于替代逐步退网的 SDH。其核心容器为 **fgODUflex(p)**（p = 1~119，标称速率约 p × 10 409.203 kbit/s，粒度约 10 Mbit/s），再通过 **fgGMP** 复用进 ODU0/1/2/flex(fgTS,n) 等服务器容器。

fgODUflex 帧为字节块结构：**4 行 × 3824 列**，由 fgODUflex 开销区、fgOPUflex 开销区和 fgOPUflex 净荷区组成：

| 区域 | 列范围 | 内容 |
| --- | --- | --- |
| fgODUflex 开销区 | 1~14、1905~1918 | FAS、MFAS、PM/TCM1/TCM2 开销、DAi 相位差累积开销 |
| fgOPUflex 开销区 | 15~16、1919~1920 | PT、CSF、映射专用开销（OMFI、JC、CFS、fgBWR RCOH 等） |
| fgOPUflex 净荷区 | 17~1904、1921~3824 | 每行 118×16B + 119×16B = 3792 字节，全帧 15168 字节 |

开销占比：每帧总长 4×3824 = 15296 字节，其中 fgODUflex OH 112 字节 + fgOPUflex OH 16 字节，共 **128 字节/帧（约 0.84%）**。

fgOTN 开销的整体清单（G.709.20 第 B.2 条）：**FAS/MFAS、PM（TTI、BIP-8、BEI、BDI、STAT、APS、DM）、2 级 TCM（TTI、BIP-8、BEI/BIAE、BDI、STAT、APS、DM）、PT、CSF、DAi，以及无损伤带宽调整相关开销**。

---

## 2. 帧定位与复帧开销（FAS / MFAS）

| 字段 | 位置 | 长度 | 说明 |
| --- | --- | --- | --- |
| FAS0~FAS7 | 行 1~4，列 1~4 与列 1905~1908 | 8 个，每个 4 字节 | 8 个 FAS 分布在半帧首尾，每行 2 个；FAS 第 4 字节 = 0x28 XOR HRN（半行号），即 0x28~0x2F |
| MFAS | 行 1，列 7 | 1 字节 | 每帧加 1，构成 256 帧复帧，用于 TTI、DM、fgTSOH、fgTSMxOH 等复帧索引 |

---

## 3. PM / TCM1 / TCM2 开销

fgODUflex 提供 1 级通道监控（PM）和 2 级串联连接监控（TCM1、TCM2），三者字段相同（BIAE 仅 TCM 有）。各字段位置如下：

| 字段 | PM | TCM1 | TCM2 | 说明 |
| --- | --- | --- | --- | --- |
| TTI（踪迹标识） | 行 1~4，列 1909~1910 | 行 1~4，列 1913~1914 | 行 1~4，列 1911~1912 | 32 字节，分 4 帧传输，由 MFAS[7:8] 索引，每帧传 8 字节 |
| BIP-8（误码检测） | 行 3，列 11 | 行 3，列 8 | 行 3，列 5 | 对第 i 帧 fgOPUflex 区计算，插入第 i+2 帧 |
| BDI（反向缺陷指示） | 每行，列 12，bit 5 | 每行，列 13，bit 5 | 每行，列 13，bit 1 | 1 bit，回传上游方向检测到的信号失效 |
| BEI / BIAE（反向误码/反向输入对齐错误指示） | 行 3，列 12，bits 1~4（BEI） | 行 3，列 9，bits 1~4 | 行 3，列 6，bits 1~4 | 4 bit；BIAE 仅用于 TCM1/TCM2 |
| STAT（状态指示） | 每行，列 12，bits 6~8 | 每行，列 13，bits 6~8 | 每行，列 13，bits 2~4 | 3 bit，指示 fgODUflex-LCK / -OCI / -AIS 等维护信号 |
| DM（时延测量） | 行 2，列 7 | 行 2，列 6 | 行 2，列 5 | 1 字节，承载 1DM / 2DMM / 2DMR 消息 |
| APS（自动保护倒换） | 行 4，列 9~10 | 行 4，列 7~8 | 行 4，列 5~6 | 2 字节，编码格式与 ODUk APS 一致（Req/State、Prot Type、Req Sgnl、Brid Sgnl） |

补充说明：

- **TTI**：32 字节结构（SAPI/DAPI/运营商特定字节等），PM/TCM1/TCM2 各 8 字节 × 4 帧。
- **DM**：三种消息——1DM（单向）、2DMM（双向测量请求）、2DMR（双向测量响应）；每条消息跨 32 帧，按 MFAS[4:8] 对齐；时间戳采用 IEEE 1588 格式（32 bit 纳秒 + 32 bit 秒）；CRC-12 校验，生成多项式 G(x) = x¹² + x¹¹ + x³ + x² + x + 1。
  - 单向时延：t = Rx-f-TS − Tx-f-TS
  - 双向时延：t = (Rx-b-TS − Tx-f-TS) − (Tx-b-TS − Rx-f-TS)
- **STAT**：编码含义沿用 G.709 表 15-5/15-7，将 ODU-LCK/OCI/AIS 替换为 fgODUflex-LCK/OCI/AIS。

---

## 4. DAi（相位差累积）开销

用于 **CBR 业务定时透明传输**：fgOTN 中间节点不做时钟低通滤波，而是逐节点测量并累积时钟相位差（PD），宿端据此恢复 CBR 客户时钟。

| 项目 | 内容 |
| --- | --- |
| 位置 | 行 1~4，列 1915~1917 |
| 数量/格式 | 4 组 DAi（i = 1~4），每组 3 字节（DAi.1、DAi.2、DAi.3），斜向分布 |
| 编码 | 8 bit 有符号二进制补码，合法范围 −127 ~ +127，bit 1 为 MSB |
| 差错保护 | DAi.2、DAi.3 为 DAi.1 的重复，宿端 3 取 2 多数判决 |
| 工作方式 | 每采样周期 Tsamp（≥ DAi 间隔，即 fgODUflex(p) 一行周期；示例值 3 ms）测量 NMSCk（归一化复用服务器时钟，标称 311.04 MHz）计数偏差 PD，累加进 DAi；超范围时必须无损处理 |

---

## 5. fgOPUflex 开销

### 5.1 PT（净荷类型）与 CSF

| 字段 | 位置 | 长度 | 说明 |
| --- | --- | --- | --- |
| PT | 行 4，列 15，bits 3~8 | 6 bit | 指示 fgOPUflex 净荷组成 |
| CSF | 行 4，列 15，bit 1 | 1 bit | 客户信号失效指示，置 1 表示客户信号失效 |
| RES | 行 4，列 15，bit 2 | 1 bit | 保留 |

fgOPUflex PT 码点（G.709 表 M.2）：

| PT（hex） | 含义 |
| --- | --- |
| 01 | 实验性映射 |
| 02 | 分组客户映射（M.5.2） |
| 03 | CBR 客户映射（M.5.3） |
| 05 | v12×VC-12 映射（M.5.4.1） |
| 07 | v23×VC-3 映射（M.5.4.1） |
| 08 | VC-4 映射（M.5.4.1） |
| 09 | v12×E1 映射（M.5.4.2） |
| 30~37 | 厂商私有保留码 |
| 3E | PRBS 测试信号映射（M.5.1，2³¹−1 PRBS） |
| 04、06、15、26、3F | 不可用（维护信号中会出现这些图案） |

### 5.2 分组业务（PKT）映射开销（PT = 02）

- **OMFI（OPU 复帧指示）**：位于行 1~4，列 16 与 1920；每个 fgOPUflex 帧加 1（值 0~10），构成 **11 帧复帧**，用于对齐 64B/66B 块（每 11 帧承载 20224 个 64B/66B 块）。
- **fgOFCS**：为分组业务增加的一种帧校验序列（CRC-32，生成多项式与以太网相同但按字节内 MSB→LSB 反向计算），用于**错误标记**；检测到错误时，将 /T/ 块前的最后一个 /D/ 块替换为 /E/ 错误控制块。
- **fgBWR RCOH**：无损伤带宽调整控制开销（见第 7 节）。
- 速率适配：通过 64B/66B 空闲控制块插入/删除（IMP）完成。

### 5.3 CBR 业务映射开销（PT = 03）

采用 GMP 映射（m = 128 bit，即 16 字节粒度）。行 1~2 与行 3~4 各构成一个独立的 2 行净荷容器，各带一组调整开销：

| 字段 | 位置 | 说明 |
| --- | --- | --- |
| JC1、JC2 | fgOPUflex 开销区（每组 6 字节） | 14 bit 计数，承载 Cm |
| JC3 | 同上 | CRC-8，校验 Cm |
| JC4~JC6（bits 4~8） | 同上 | 承载累积 CnD(t)（k = 10） |
| JC6（CRC-5） | 同上 | 校验 JC4/JC5 的 CnD 编码，G(x) = x⁵ + x + 1 |

### 5.4 VC-n / E1 业务映射开销（PT = 05、07、08、09）

VC-12/VC-3/VC-4 及 E1 均采用 GMP 映射，但 **CnD 不激活**（JC4~JC6 的 bits 4~8 保留），并增加 **CFS（客户帧起始）** 开销：

| 字段 | 位置 | 说明 |
| --- | --- | --- |
| JC1~JC3 | fgOPUflex 开销区（每组） | Cm 计数（14 bit）+ CRC-8 |
| JC4~JC6 | 同上 | 仅保留（无 CnD） |
| CFS | 行 1：列 14 bits 3~8 + 列 15 bits 1~3；行 3 同位置 | 9 bit，指示本 2 行净荷容器中首个客户帧起点之前的 Data 块数；范围 0 ~ Cm−1；容器内无客户帧起点时置 0x1FF |

E1 映射（PT = 09）：多个 E1（v12 = 1~4）先异步映射进 VC-12、再同步映射进 TU-12，合并后经 GMP 映射进 fgOPUflex(1)；开销包含合并的 CSF、两组 JC1~JC6 及两组 CFS。

---

## 6. fgODUflex 维护信号

| 信号 | 图案（除 FAS0~FAS7 与 MFAS 外的整个信号） |
| --- | --- |
| fgODUflex-AIS | 全 1 |
| fgODUflex-OCI | 重复 “0110 0110” |
| fgODUflex-LCK | 重复 “0101 0101” |

通过 PM/TCM1/TCM2 的 STAT 位检测；AIS/OCI/LCK 插入点与接口之间可附加 1~2 级 TCM 和/或 APS 开销。

---

## 7. 无损伤带宽调整（Hitless Resizing）控制开销（Annex O）

fgODUflex 带宽调整粒度约 10 Mbit/s，采用**单步跳变**（非斜坡）机制，秒级完成。控制开销分两层：

### 7.1 fgLCR RCOH（链路连接调整控制开销）

位于 **fgTSMxOH 的 bits 13~18**（见第 8 节），随每个 fgTS 传输，共 6 bit：

| 字段 | 长度 | 含义 |
| --- | --- | --- |
| RP | 1 bit | 调整协议使能：1 = 该 fgTS 上调整协议激活；0 = 未激活/结束 |
| TSCC | 1 bit | 连通性检查使能：1 = 检查 OPU 复用段链路连接与 fgODUflex 连接的连通性 |
| CTRL | 2 bit | 调整命令：00 IDLE、01 ADD（请求增加 fgTS）、10 REMOVE（请求删除 fgTS）、11 保留 |
| TSGS | 2 bit | 应答：00 IDLE、01 ACCEPT（接受）、10 REJECT（拒绝）、11 保留 |

TPID（端口标识，用于指明被增删的 fgTS 归属）复用 fgTSMxOH 中的 **MSI** 字段。

### 7.2 fgBWR RCOH（fgODUflex 带宽调整控制开销）

位于 **fgOPUflex 开销区行 1~3、列 15**，共 5 bit：

| 字段 | 长度 | 含义 |
| --- | --- | --- |
| BWR_IND | 1 bit | 0→1：单步速率跳变指示（fgODUflex 速率、服务器 fgODTU 由 M 个 fgTS 变为 M±N 个 fgTS、时隙重新分配三者同步完成）；1→0：调整结束 |
| NCS | 1 bit | 0→1：宿端完成调整的确认；1→0：调整过程结束确认 |
| CRC-3 | 3 bit | 对行 1~2 bits 1~3 的校验（复用 G.7044 的 CRC-3） |

### 7.3 调整流程要点

1. 源节点置 RP=1，逐段（link by link）在待增/删 fgTS 的 fgLCR 开销中发 CTRL=ADD/REMOVE + TPID，全部发出后置 TSCC=1；
2. 宿端就绪后反向回 TSGS=ACCEPT，否则回 REJECT 并回退；
3. 源节点经 fgBWR 开销发 BWR_IND=1，在指定位置（行 3 列 1904 bit 8 之后）完成 fgODUflex 单步速率跳变；
4. 各复用段在指定位置 Y 将 fgODTU 从 M 个 fgTS 切换为 M±N 个 fgTS；
5. 宿端完成调整后回 NCS=1；源节点发 BWR_IND=0、RP=0 结束流程。

---

## 8. 服务器层开销（fgOTN 复用进 OPU）

fgODUflex 经 fgGMP 映射进 **fgODTU.M**，再以 **16 字节交织**映射进一个或多个 OPU **细粒度时隙 fgTS**。fgTS 粒度略高于 10 Mbit/s：

| 服务器 OPU | fgTS 数量 | fgTSOH 复帧 | fgTSMxOH 复帧 |
| --- | --- | --- | --- |
| OPU0(fgTS) | 119 | 32 帧（MFAS[4:8]） | 256 帧（MFAS） |
| OPU1(fgTS) | 238 | 64 帧（MFAS[3:8]） | 2×256 帧（MFAS + OMFI） |
| OPU2(fgTS) | 952 | 256 帧（MFAS[1:8]） | 8×256 帧（MFAS + OMFI） |
| OPUflex(fgTS,n)，n=3~7 | n×119 | 32n 帧（MFAS[4:8] + OMFI） | n×256 帧（MFAS + OMFI） |

### 8.1 fgTSOH（细粒度时隙开销，12 bit）

- 每个 fgTS 在 32 帧集合中出现一次 fgTSOH：帧 #1~#29 各携带 4 个，帧 #30 携带 3 个，帧 #31~#32 不携带。
- 内容：**fgODTU.M 的调整开销**（fgGMP justification overhead）：
  - CmT：6 bit（C1~C6），为 Cm 减去基准值 CmB 后的差值计数；
  - II：1 bit 增指示；DI：1 bit 减指示（按位反转图案指示 ±1 变化）；
  - CRC-4：4 bit，G(x) = x⁴ + x² + 1，覆盖 C1~C6、II、DI。

### 8.2 fgODTU.M

- 净荷区：128 行 × 2M 个 16 字节块（共 256M 块）；fgODTU.M 承载于 M 个 fgTS。
- 开销区：12 bit，位于所占用**最后一个 fgTS 的 fgTSOH**。
- fgGMP：简化 GMP，16 字节数据/填充粒度，无需 CnD 传递；CmT 每个 fgODTU.M 复帧更新一次。

### 8.3 fgTSMxOH（fgTS 复用开销）

- 位置：行 4，列 15~16；每帧同时携带 2 个 fgTSMxOH（分别对应相邻两个 fgTS）；每个 fgTSMxOH 跨 4 帧传完（每帧 6 bit，共 24 bit）。
- 格式（24 bit）：前 18 bit 为信息位 + 后 6 bit 为 **CRC-6**（G(x) = x⁶ + x³ + x² + 1）：
  - **OCCU**：fgTS 占用指示（0 = 未分配）；
  - **MSI**（复用结构标识）：指示该 fgTS 承载的 fgODTU.ts 端口号（1~952），支持端口到 fgTS 的灵活分配；fgTS 未分配时 MSI 置全 1；MSI 同时兼任无损伤调整中的 TPID；
  - **fgLCR RCOH**（bits 13~18）：RP、TSCC、CTRL、TSGS（见 7.1）。
- 复帧索引：OPU0 由 MFAS 索引（256 帧）；OPUflex/OPU1/OPU2 由 MFAS + OMFI 索引（n×256 帧）。

### 8.4 OPU 级 OMFI

- 位置：行 4，列 16，bits 5~8（4 bit）。
- 每 256 帧加 1，范围 0 ~ n−1，构成 n×256 帧的 fgTSMxOH 复帧；OPU0 的 OMFI 恒为 0。
- 注意与 5.2 中分组映射的 fgOPUflex OMFI（11 帧复帧）用途不同。

---

## 9. 开销位置速查表

| 开销 | 位置（行，列） | 长度 | 功能 |
| --- | --- | --- | --- |
| FAS0~FAS7 | 1~4，1~4 与 1905~1908 | 8×4 B | 帧定位 |
| MFAS | 1，7 | 1 B | 复帧定位（256 帧） |
| PM TTI | 1~4，1909~1910 | 8 B（4 帧 32 B） | 路径踪迹识别 |
| TCM2 TTI | 1~4，1911~1912 | 8 B | 级联监控踪迹 |
| TCM1 TTI | 1~4，1913~1914 | 8 B | 级联监控踪迹 |
| PM/TCM1/TCM2 BIP-8 | 3，11 / 3，8 / 3，5 | 各 1 B | 误码检测 |
| PM/TCM1/TCM2 BDI | 各行列 12 bit5 / 13 bit5 / 13 bit1 | 各 1 bit | 反向缺陷指示 |
| PM/TCM1/TCM2 BEI/BIAE | 3，12 bits1-4 / 3，9 bits1-4 / 3，6 bits1-4 | 各 4 bit | 反向误码/对齐错误 |
| PM/TCM1/TCM2 STAT | 各行列 12 bits6-8 / 13 bits6-8 / 13 bits2-4 | 各 3 bit | 维护信号状态 |
| PM/TCM1/TCM2 DM | 2，7 / 2，6 / 2，5 | 各 1 B | 时延测量 |
| PM/TCM1/TCM2 APS | 4，9-10 / 4，7-8 / 4，5-6 | 各 2 B | 保护倒换 |
| DA1~DA4 | 1~4，1915~1917（斜向） | 4×3 B | 时钟相位差累积 |
| fgOPUflex PT/CSF | 4，15 | 6+1 bit | 净荷类型/客户失效 |
| OMFI（分组映射） | 1~4，16 与 1920 | 8 B | 11 帧复帧 |
| JC1~JC6（CBR/VC/E1） | fgOPUflex 开销区 | 2×6 B | GMP 调整（Cm/CnD/CRC） |
| CFS（VC/E1） | 1 与 3，列 14 bits3-8 + 列 15 bits1-3 | 2×9 bit | 客户帧起始 |
| fgBWR RCOH | 1~3，15 | 5 bit | 无损伤带宽调整 |
| fgTSOH / fgODTU.M OH | 最后一个 fgTS | 12 bit | fgGMP 调整（CmT/II/DI/CRC-4） |
| fgTSMxOH（OCCU/MSI/fgLCR RCOH/CRC-6） | 4，15~16（复帧） | 24 bit/个 | fgTS 复用标识与调整控制 |

---

## 10. 参考来源

- [ITU-T G.709.20 (04/2024) Overview of fine grain OTN](https://www.itu.int/rec/T-REC-G.709.20-202404-I/en)
- [ITU-T G.709.20 (2024) Amd.1 (05/2025)](https://www.itu.int/dms_pubrec/itu-t/rec/g/T-REC-G.709.20-202505-I!Amd1!TOC-HTM-E.htm)
- [ITU-T G.709/Y.1331 (2020) 及 Amd.4 (07/2025)，Annex M/N/O](https://www.itu.int/rec/T-REC-G.709-202507-I/en)
- [电信科学《fgOTN 关键技术组网应用研究》](https://www.telecomsci.com/rc-pub/front/front-article/download/82012130/lowqualitypdf/fgOTN%E5%85%B3%E9%94%AE%E6%8A%80%E6%9C%AF%E5%92%8C%E7%BB%84%E7%BD%91%E5%BA%94%E7%94%A8%E7%A0%94%E7%A9%B6.pdf)

> 注：字段位置以 ITU-T G.709 Amd.4 的 Annex M/N/O 正文为准；如需具体位图（如 APS 编码、FAS 图案、OMFI 布局），建议直接查阅 G.709 原文对应图（Figure M.2~M.13、N.1~N.7、O.1A/O.1B）。
