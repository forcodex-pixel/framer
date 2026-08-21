// 多流输入 monitor：每拍在 posedge（active 区，NBA 前）直读 7 组输入端口
// （vld && rdy），按流上报。active 区读到沿前值，与 KOA 在同沿入队的报文一致；
// rdy 为组合（段未满且本拍接受），同一拍可多条流同时握手（5 路 SBUF 并行吸收），
// 逐流上报。未接受流 rdy=0、vld 保持，下一拍再采（不丢）。
// 反压统计：vld && !rdy（仅统计，不影响 scoreboard）。
class koa_in_monitor extends uvm_monitor;
    uvm_analysis_port #(koa_item) ap;
    `uvm_component_utils(koa_in_monitor)

    function new(string name = "koa_in_monitor", uvm_component parent = null);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    task run_phase(uvm_phase phase);
        int n_oh = (ko_pkg::g_tb_cfg.n_oh_planes > 0) ? ko_pkg::g_tb_cfg.n_oh_planes : 1;
        int n_x2x = (ko_pkg::g_tb_cfg.n_x2x_planes > 0) ? ko_pkg::g_tb_cfg.n_x2x_planes : 1;
        forever begin
            @(posedge ko_pkg::g_tb_cfg.vif.clk);
            // 反压统计：vld 有效但 rdy=0
            begin
                bit bp[7];
                bp[0] = (|ko_pkg::g_tb_cfg.vif.oh_e_vld) && !ko_pkg::g_tb_cfg.vif.oh_e_rdy[0];
                bp[1] = (|ko_pkg::g_tb_cfg.vif.oh_i_vld) && !ko_pkg::g_tb_cfg.vif.oh_i_rdy[0];
                bp[2] = (|ko_pkg::g_tb_cfg.vif.aps_e_vld) && !ko_pkg::g_tb_cfg.vif.aps_e_rdy[0];
                bp[3] = (|ko_pkg::g_tb_cfg.vif.aps_i_vld) && !ko_pkg::g_tb_cfg.vif.aps_i_rdy[0];
                bp[4] = (|ko_pkg::g_tb_cfg.vif.alm_vld) && !ko_pkg::g_tb_cfg.vif.alm_rdy[0];
                bp[5] = ko_pkg::g_tb_cfg.vif.u_e_vld && !ko_pkg::g_tb_cfg.vif.u_e_rdy;
                bp[6] = ko_pkg::g_tb_cfg.vif.u_i_vld && !ko_pkg::g_tb_cfg.vif.u_i_rdy;
                for (int f = 0; f < 7; f++) begin
                    if (bp[f]) begin
                        ko_pkg::g_tb_cfg.bp_clks[f]++;
                        if (!ko_pkg::g_tb_cfg.bp_prev[f]) ko_pkg::g_tb_cfg.bp_events[f]++;
                    end
                    ko_pkg::g_tb_cfg.bp_prev[f] = bp[f];
                end
            end
            // OH_EXT
            for (int i = 0; i < n_oh; i++)
                if (ko_pkg::g_tb_cfg.vif.oh_e_vld[i] && ko_pkg::g_tb_cfg.vif.oh_e_rdy[i])
                    sample(ST_OH_EXT, i, ko_pkg::g_tb_cfg.vif.oh_e_pri[i*3 +: 3],
                    ko_pkg::g_tb_cfg.vif.oh_e_cid[i*17 +: 17],
                    ko_pkg::g_tb_cfg.vif.oh_e_pos[i*3 +: 3]);
            // OH_INS
            for (int i = 0; i < n_oh; i++)
                if (ko_pkg::g_tb_cfg.vif.oh_i_vld[i] && ko_pkg::g_tb_cfg.vif.oh_i_rdy[i])
                    sample(ST_OH_INS, i, ko_pkg::g_tb_cfg.vif.oh_i_pri[i*3 +: 3],
                    ko_pkg::g_tb_cfg.vif.oh_i_cid[i*17 +: 17],
                    ko_pkg::g_tb_cfg.vif.oh_i_pos[i*3 +: 3]);
            // APS_EXT
            for (int i = 0; i < n_x2x; i++)
                if (ko_pkg::g_tb_cfg.vif.aps_e_vld[i] && ko_pkg::g_tb_cfg.vif.aps_e_rdy[i])
                    sample(ST_APS_EXT, i, ko_pkg::g_tb_cfg.vif.aps_e_pri[i*3 +: 3],
                    ko_pkg::g_tb_cfg.vif.aps_e_cid[i*17 +: 17],
                    ko_pkg::g_tb_cfg.vif.aps_e_pos[i*3 +: 3]);
            // APS_INS
            for (int i = 0; i < n_x2x; i++)
                if (ko_pkg::g_tb_cfg.vif.aps_i_vld[i] && ko_pkg::g_tb_cfg.vif.aps_i_rdy[i])
                    sample(ST_APS_INS, i, ko_pkg::g_tb_cfg.vif.aps_i_pri[i*3 +: 3],
                    ko_pkg::g_tb_cfg.vif.aps_i_cid[i*17 +: 17],
                    ko_pkg::g_tb_cfg.vif.aps_i_pos[i*3 +: 3]);
            // ALM
            for (int i = 0; i < n_x2x; i++)
                if (ko_pkg::g_tb_cfg.vif.alm_vld[i] && ko_pkg::g_tb_cfg.vif.alm_rdy[i])
                    sample(ST_ALM, i, ko_pkg::g_tb_cfg.vif.alm_pri[i*3 +: 3],
                    ko_pkg::g_tb_cfg.vif.alm_cid[i*17 +: 17],
                    ko_pkg::g_tb_cfg.vif.alm_pos[i*3 +: 3]);
            // UART_EXT
            if (ko_pkg::g_tb_cfg.vif.u_e_vld && ko_pkg::g_tb_cfg.vif.u_e_rdy)
                sample(ST_UART_EXT, 0, ko_pkg::g_tb_cfg.vif.u_e_pri,
                17'd0, 3'd0);
            // UART_INS
            if (ko_pkg::g_tb_cfg.vif.u_i_vld && ko_pkg::g_tb_cfg.vif.u_i_rdy)
                sample(ST_UART_INS, 0, ko_pkg::g_tb_cfg.vif.u_i_pri,
                17'd0, 3'd0);
        end
    endtask

    function void sample(koa_stream_t s, int pl, logic [2:0] pri,
        logic [16:0] cid, logic [2:0] pos);
        koa_item it = koa_item::type_id::create("it");
        it.stream = s;
        it.plane = pl;
        it.cid = cid;
        it.pos = pos;
        it.pri = pri;
        it.ev_time = $time; // 沿（active 区）采样，即 KOA 入队沿；out 用 $time-1 对齐
        it.is_out = 1'b0;
        ap.write(it);
    endfunction
endclass

// 输出 monitor：采样 out_vld，携带 out_src（0=EXT 1=INS 2=ALM 3=UART_EXT 4=UART_INS）
class koa_out_monitor extends uvm_monitor;
    uvm_analysis_port #(koa_item) ap;
    `uvm_component_utils(koa_out_monitor)

    function new(string name = "koa_out_monitor", uvm_component parent = null);
        super.new(name, parent);
        ap = new("ap", this);
    endfunction

    task run_phase(uvm_phase phase);
        forever begin
            @(posedge ko_pkg::g_tb_cfg.vif.clk);
            #1;
            if (ko_pkg::g_tb_cfg.vif.out_vld) begin
                koa_item it = koa_item::type_id::create("it");
                it.sbuf = ko_pkg::g_tb_cfg.vif.out_src;
                it.plane = 0;
                it.ev_time = $time - 1; // 输出寄存器在沿更新，对齐 DUT 输出拍
                it.is_out = 1'b1;
                it.pri = ko_pkg::g_tb_cfg.vif.out_pri;
                ap.write(it);
            end
        end
    endtask
endclass
