// 业务源输入 driver：3 个业务源各一个实例（stream_group：0=fgOTN 开销、
// 1=X2X、2=串口），负责驱动该业务源下的所有流端口（valid/ready 握手）。
// 只有不同业务源之间独立（最大 3 路并发 vld）；同一业务源内的多流
// （如 OH_EXT/OH_INS）不独立——挂同一 sequencer，item 串行到达。
//
// 握手时序（与 KOA/monitor 对齐的标准 NBA 握手）：
//   @(posedge clk) 置 vld（非阻塞，沿后生效）→ 下一沿 KOA 采到 vld=1 入队、
//   driver 采到 rdy=1（组合）→ 沿后撤 vld。monitor 在沿（active 区）读
//   vld&&rdy 沿前值，与 KOA 同拍 → 上报事件与 DUT 实际接收一致。
// KOA 仲裁每拍只收 1 条并把选中流 rdy 拉高，未选中流 rdy=0、vld 保持 → 不丢。
class koa_driver extends uvm_driver #(koa_item);
    `uvm_component_utils(koa_driver)

    int stream_group; // 0=fgOTN 1=X2X 2=UART（由 agent 设置）

    function new(string name = "koa_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    task run_phase(uvm_phase phase);
        koa_item req;
        forever begin
            seq_item_port.get_next_item(req);
            while (!ko_pkg::g_reset_done) @(posedge ko_pkg::g_tb_cfg.vif.clk);
            @(posedge ko_pkg::g_tb_cfg.vif.clk);
            case (req.stream)
                ST_OH_EXT: begin
                    if (stream_group != 0) begin
                        `uvm_error("DRV", "OH_EXT item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.oh_e_vld <= (1 << req.plane);
                        ko_pkg::g_tb_cfg.vif.oh_e_pri <= (req.pri << (req.plane * 3));
                        ko_pkg::g_tb_cfg.vif.oh_e_cid <= (req.cid << (req.plane * 17));
                        ko_pkg::g_tb_cfg.vif.oh_e_pos <= (req.pos << (req.plane * 3));
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.oh_e_rdy[req.plane] !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.oh_e_vld <= '0;
                    end
                end
                ST_OH_INS: begin
                    if (stream_group != 0) begin
                        `uvm_error("DRV", "OH_INS item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.oh_i_vld <= (1 << req.plane);
                        ko_pkg::g_tb_cfg.vif.oh_i_pri <= (req.pri << (req.plane * 3));
                        ko_pkg::g_tb_cfg.vif.oh_i_cid <= (req.cid << (req.plane * 17));
                        ko_pkg::g_tb_cfg.vif.oh_i_pos <= (req.pos << (req.plane * 3));
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.oh_i_rdy[req.plane] !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.oh_i_vld <= '0;
                    end
                end
                ST_APS_EXT: begin
                    if (stream_group != 1) begin
                        `uvm_error("DRV", "APS_EXT item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.aps_e_vld <= (1 << req.plane);
                        ko_pkg::g_tb_cfg.vif.aps_e_pri <= (req.pri << (req.plane * 3));
                        ko_pkg::g_tb_cfg.vif.aps_e_cid <= (req.cid << (req.plane * 17));
                        ko_pkg::g_tb_cfg.vif.aps_e_pos <= (req.pos << (req.plane * 3));
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.aps_e_rdy[req.plane] !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.aps_e_vld <= '0;
                    end
                end
                ST_APS_INS: begin
                    if (stream_group != 1) begin
                        `uvm_error("DRV", "APS_INS item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.aps_i_vld <= (1 << req.plane);
                        ko_pkg::g_tb_cfg.vif.aps_i_pri <= (req.pri << (req.plane * 3));
                        ko_pkg::g_tb_cfg.vif.aps_i_cid <= (req.cid << (req.plane * 17));
                        ko_pkg::g_tb_cfg.vif.aps_i_pos <= (req.pos << (req.plane * 3));
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.aps_i_rdy[req.plane] !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.aps_i_vld <= '0;
                    end
                end
                ST_ALM: begin
                    if (stream_group != 1) begin
                        `uvm_error("DRV", "ALM item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.alm_vld <= (1 << req.plane);
                        ko_pkg::g_tb_cfg.vif.alm_pri <= (req.pri << (req.plane * 3));
                        ko_pkg::g_tb_cfg.vif.alm_cid <= (req.cid << (req.plane * 17));
                        ko_pkg::g_tb_cfg.vif.alm_pos <= (req.pos << (req.plane * 3));
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.alm_rdy[req.plane] !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.alm_vld <= '0;
                    end
                end
                ST_UART_EXT: begin
                    if (stream_group != 2) begin
                        `uvm_error("DRV", "UART_EXT item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.u_e_vld <= 1'b1;
                        ko_pkg::g_tb_cfg.vif.u_e_pri <= req.pri;
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.u_e_rdy !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.u_e_vld <= 1'b0;
                    end
                end
                default: begin
                    if (stream_group != 2) begin
                        `uvm_error("DRV", "UART_INS item 挂到错误的业务源 driver")
                    end else begin
                        ko_pkg::g_tb_cfg.vif.u_i_vld <= 1'b1;
                        ko_pkg::g_tb_cfg.vif.u_i_pri <= req.pri;
                        do @(posedge ko_pkg::g_tb_cfg.vif.clk);
                        while (ko_pkg::g_tb_cfg.vif.u_i_rdy !== 1'b1);
                        ko_pkg::g_tb_cfg.vif.u_i_vld <= 1'b0;
                    end
                end
            endcase
            seq_item_port.item_done();
            @(posedge ko_pkg::g_tb_cfg.vif.clk); // 一拍空闲，保证 vld 撤/置稳定
        end
    endtask
endclass
