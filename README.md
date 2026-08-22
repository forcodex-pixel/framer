# framer —— fgOTN KO / POE 验证与方案工作区

## 目录结构

```text
framer/
├─ docs/                    # 方案文档与设计图
│  ├─ POE系统设计.md         # 完整设计文档（当前版：KOA + POE 全链路）
│  └─ archive/              # 历史设计文档与旧版 drawio 图
└─ uvm_poe/                 # 主工程：KOA 调度 + POE THM/th_sch/burst_sch/CU/dma_ctrl
```

## 主工程：uvm_poe

KOA 8 组 RR+SP 调度 → POE THM（线程管理/保序/CSR/锁）→ th_sch（一级发射：
q0/q1 仅存 i/v，c_task 解析入独立缓存）→ burst_sch（二级发射：q0→EU0、q1→EU1，
4 个 DSE 调度器选 c_task）→ EU×2（各 4 个 CU 桩）+ dma_ctrl×4（C 窗每线程独享 8 位置）。

运行：

```bash
cd uvm_poe/sim
TESTNAME=koa_smoke_test RUN_US=30 N_OH_PLANES=4 OH_SLOTS=9520 \
  N_X2X_PLANES=8 X2X_SLOTS=9520 N_CH=8 UART_MPPS=60 WAVE=1 ./run_verilator.sh
```

完整方案见 `docs/POE系统设计.md`；历史文档在 `docs/archive/`。
