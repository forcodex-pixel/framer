// 串口指令源序列（UART_EXT / UART_INS）：每路 ≤60 Mpps，每拍伯努利随机到达
// dir：0=EXT（pri=7），1=INS（pri=随机 0..7）
class koa_uart_seq extends uvm_sequence #(koa_item);
    `uvm_object_utils(koa_uart_seq)
    int dir; // 0=ST_UART_EXT，1=ST_UART_INS

    function new(string name = "koa_uart_seq");
        super.new(name);
        dir = 0;
    endfunction

    task body();
        koa_tb_config cfg;
        int pct; // 每拍发射概率（千分比）
        longint start_t;
        cfg = ko_pkg::g_tb_cfg;
        while (!ko_pkg::g_reset_done) @(posedge cfg.vif.clk);

        pct = int'(cfg.uart_mpps * 1e6 / cfg.clk_freq_hz * 1000.0); // e.g. 60M/1G*1000=60
        if (pct > 1000) pct = 1000;
        start_t = $time;

        `uvm_info("USEQ", $sformatf("串口 %s: %.2f Mpps（每拍概率 %d/1000），窗口 %0.1f us",
        (dir == 0) ? "UART_EXT" : "UART_INS", cfg.uart_mpps, pct, cfg.run_us), UVM_LOW)

        while (($time - start_t) < cfg.run_us * 1000.0) begin
            @(posedge cfg.vif.clk);
            if (($urandom % 1000) < pct) begin
                koa_item it = koa_item::type_id::create("it");
                if (dir == 0)
                    it.pri = 3'd7; // UART_EXT：固定 7
                it.stream = (dir == 0) ? ST_UART_EXT : ST_UART_INS;
                it.plane = 0;
                start_item(it);
                finish_item(it);
            end
        end
    endtask
endclass
