`uvm_analysis_imp_decl(_in)
`uvm_analysis_imp_decl(_out)

// scoreboard：KOA 输出参考模型（5×SBUF × 8 优先级段 + SP/RR）
// - 输入事件按 (SBUF, pri) 入对应段（stream 映射 SBUF：EXT=OH_EXT/APS_EXT、
//   INS=OH_INS/APS_INS、ALM、UART_EXT、UART_INS），每段 FIFO
// - 输出事件：SP 选最高非空 pri 组（组号最小），组内 5 个 SBUF 按 rr_ptr 轮询
//   （rr_ptr 每拍推进，取最先非空段）出队；比对组号/数据
// - 保序由 THM 侧负责（KOA 不保序），本 scoreboard 不校验保序键
class koa_scoreboard extends uvm_scoreboard;
    uvm_analysis_imp_in #(koa_item, koa_scoreboard) in_imp;
    uvm_analysis_imp_out #(koa_item, koa_scoreboard) out_imp;

    localparam int N_SBUF = 5;
    localparam int N_PRI = 8;
    localparam int MAXQ = 512; // 每个 (SBUF,pri) 段深度（≥KOA 段深 320）

    koa_item all_ev[$];
    koa_item q_items[N_SBUF][N_PRI][MAXQ];
    int q_head[N_SBUF][N_PRI], q_tail[N_SBUF][N_PRI], q_cnt[N_SBUF][N_PRI];
    int rr_ptr;
    int peak_q_occ;
    int in_cnt, out_cnt;
    int n_mismatch;
    `uvm_component_utils(koa_scoreboard)

    function new(string name = "koa_scoreboard", uvm_component parent = null);
        super.new(name, parent);
        in_imp = new("in_imp", this);
        out_imp = new("out_imp", this);
    endfunction

    function void write_in(koa_item it);
        if (it.is_out) return;
        all_ev.push_back(it);
        in_cnt++;
    endfunction

    function void write_out(koa_item it);
        if (!it.is_out) return;
        all_ev.push_back(it);
        out_cnt++;
    endfunction

    // stream → SBUF 映射（与 KOA 一致）
    function int stream_sbuf(koa_stream_t s);
        case (s) inside
            ST_OH_EXT, ST_APS_EXT: return 0; // EXT_SBUF
            ST_OH_INS, ST_APS_INS: return 1; // INS_SBUF
            ST_ALM: return 2; // ALM_SBUF
            ST_UART_EXT: return 3;
            default: return 4; // UART_INS
        endcase
    endfunction

    // 段内写入顺序（与 KOA 合并向量写一致）：APS 类固定靠前，OH 类后写；
    // 同类内保持 monitor 采样顺序（即平面编号小优先）
    function int wr_order(koa_stream_t s);
        case (s) inside
            ST_OH_EXT, ST_OH_INS: return 1;
            default: return 0;
        endcase
    endfunction

    function void check_phase(uvm_phase phase);
        string fnames[7];
        longint cur_t;
        fnames = '{"OH_EXT","OH_INS","APS_EXT","APS_INS","ALM","UART_EXT","UART_INS"};
        // 按时间戳稳定排序；同一时刻 OUT（KOA 输出）排在 IN（入队）之前，
        // 复现 KOA 同拍"先读队首出队、后写入队"的语义
        for (int i = 1; i < all_ev.size(); i++) begin
            koa_item key = all_ev[i];
            int j = i - 1;
            while (j >= 0 && (all_ev[j].ev_time > key.ev_time ||
                (all_ev[j].ev_time == key.ev_time &&
                    key.is_out && !all_ev[j].is_out))) begin
                all_ev[j+1] = all_ev[j];
                j--;
            end
            all_ev[j+1] = key;
        end
        // 模拟：按拍推进 rr_ptr（KOA 每拍推进；拍 = ev_time，1ns/拍）
        // rr 从复位起每拍推进，故初始化为第一个事件时刻的拍数模 5（与 KOA 对齐）
        cur_t = (all_ev.size() > 0) ? longint'(all_ev[0].ev_time) : -1;
        rr_ptr = (all_ev.size() > 0) ? int'(all_ev[0].ev_time % N_SBUF) : 0;
        for (int i = 0; i < all_ev.size();) begin
            if (all_ev[i].ev_time != cur_t) begin
                if (cur_t != -1) rr_ptr = (rr_ptr + (all_ev[i].ev_time - cur_t)) % N_SBUF;
                cur_t = all_ev[i].ev_time;
            end
            if (!all_ev[i].is_out) begin
                // 收集同一 ev_time 的全部 IN（排序后连续），按 KOA 写入顺序重排后入队：
                // 同段同拍多条时 DUT 按 APS 类先、OH 类后合并写入，FIFO 内顺序必须一致
                int n = 0;
                while (i + n < all_ev.size() && !all_ev[i+n].is_out && all_ev[i+n].ev_time == cur_t) n++;
                for (int a = 1; a < n; a++) begin
                    koa_item key = all_ev[i+a];
                    int j = a - 1;
                    while (j >= 0 && wr_order(all_ev[i+j].stream) > wr_order(key.stream)) begin
                        all_ev[i+j+1] = all_ev[i+j];
                        j--;
                    end
                    all_ev[i+j+1] = key;
                end
                for (int a = 0; a < n; a++) begin
                    int s = stream_sbuf(all_ev[i+a].stream);
                    int g = all_ev[i+a].pri;
                    if (q_cnt[s][g] == MAXQ) begin
                        `uvm_error("SCB", $sformatf("SBUF%0d 优先级段 %0d 满（输入超限）", s, g))
                        n_mismatch++;
                    end else begin
                        q_items[s][g][q_tail[s][g]] = all_ev[i+a];
                        q_tail[s][g] = (q_tail[s][g] == MAXQ-1) ? 0 : q_tail[s][g] + 1;
                        q_cnt[s][g] = q_cnt[s][g] + 1;
                        if (q_cnt[s][g] > peak_q_occ) peak_q_occ = q_cnt[s][g];
                    end
                end
                i += n;
            end else begin
                // SP：最高非空 pri 组（任一 SBUF 的该段非空）
                int g = -1;
                int s_sel;
                for (int p = 0; p < N_PRI; p++) begin
                    for (int s = 0; s < N_SBUF; s++)
                    if (q_cnt[s][p] != 0) begin
                        g = p;
                        break;
                    end
                    if (g != -1) break;
                end
                if (g == -1) begin
                    `uvm_error("SCB", $sformatf("输出 KO 时所有优先级队列为空（@%0t）", all_ev[i].ev_time))
                    n_mismatch++;
                    continue;
                end
                // 组内 RR：从 rr_ptr 开始找第一个非空 SBUF 段
                s_sel = rr_ptr;
                for (int k = 0; k < N_SBUF; k++)
                if (q_cnt[(rr_ptr + k) % N_SBUF][g] != 0) begin
                    s_sel = (rr_ptr + k) % N_SBUF;
                    break;
                end
                if (all_ev[i].sbuf !== g)
                    `uvm_error("SCB", $sformatf("优先级组不符：输出组%0d 期望组%0d（@%0t）",
                    all_ev[i].sbuf, g, all_ev[i].ev_time))
                    n_mismatch += (all_ev[i].sbuf !== g);
                q_head[s_sel][g] = (q_head[s_sel][g] == MAXQ-1) ? 0 : q_head[s_sel][g] + 1;
                q_cnt[s_sel][g] = q_cnt[s_sel][g] - 1;
                i++;
            end
        end
        if (in_cnt != out_cnt)
            `uvm_error("SCB", $sformatf("数量不守恒：输入=%0d 输出=%0d", in_cnt, out_cnt))
            for (int p = 0; p < N_PRI; p++)
                for (int s = 0; s < N_SBUF; s++)
                    if (q_cnt[s][p] != 0)
                        `uvm_error("SCB", $sformatf("SBUF%0d 优先级段 %0d 未清空：%0d 条", s, p, q_cnt[s][p]))
                        `uvm_info("SCB", $sformatf("输入=%0d 输出=%0d，错配=%0d，优先级队列峰值占用=%0d",
                        in_cnt, out_cnt, n_mismatch, peak_q_occ), UVM_LOW)
                        for (int f = 0; f < 7; f++)
                            if (ko_pkg::g_tb_cfg.bp_clks[f] != 0)
                                `uvm_info("SCB", $sformatf("反压[%0s]: 事件%0d 次，共%0d 拍（%.1f%% 窗口）",
                                fnames[f], ko_pkg::g_tb_cfg.bp_events[f], ko_pkg::g_tb_cfg.bp_clks[f],
                                100.0 * ko_pkg::g_tb_cfg.bp_clks[f] / (ko_pkg::g_tb_cfg.run_us * 1000.0)),
                                UVM_LOW)
                            endfunction
                        endclass
