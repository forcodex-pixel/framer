// fgOTN 开销源序列（OH_EXT / OH_INS）：4 平面，每平面随机通道时隙表
// （各通道时隙可不同，如 1+1+9518；每通道独立帧周期/带宽优先级）
// dir：0=EXT（pri=7），1=INS（pri=带宽：1..20 时隙→1，>20→0）
class koa_oh_seq extends uvm_sequence #(koa_item);
    `uvm_object_utils(koa_oh_seq)
    int dir; // 0=ST_OH_EXT，1=ST_OH_INS

    function new(string name = "koa_oh_seq");
        super.new(name);
        dir = 0;
    endfunction

    function int oh_off(int k);
        if (k > 7) k = 7;
        return (k / 2) * 3824 + ((k % 2 == 0) ? 0 : 1904);
    endfunction

    // 随机生成每平面通道时隙表：n_ch 个通道，总和 total（每通道 ≥1）
    function automatic void gen_slots(int total, int n, ref int arr[]);
        int rem = total - n;
        arr = new[n];
        for (int c = 0; c < n - 1; c++) begin
            int give = $urandom % (rem + 1);
            arr[c] = 1 + give;
            rem -= give;
        end
        arr[n-1] = 1 + rem;
    endfunction

    task body();
        koa_tb_config cfg;
        real skew, run_s, cursor, t;
        int n_pl, max_ch;
        int n_ch; // 本平面实际通道数
        int slots_arr[];
        real t_arr[];
        int plane_arr[], chan_arr[], nfg_arr[], k_arr[];
        int pri_arr[];
        int n_ev;

        cfg = ko_pkg::g_tb_cfg;
        while (!ko_pkg::g_reset_done) @(posedge cfg.vif.clk);

        n_pl = (cfg.n_oh_planes > 0) ? cfg.n_oh_planes : 1;
        max_ch = (cfg.n_ch_per_plane > 0) ? cfg.n_ch_per_plane : 1;
        skew = cfg.oh_plane_skew ? 0.0 : 0.0; // 平面错开在通道相位中体现
        run_s = cfg.run_us * 1e-6;
        cursor = 0.0;
        n_ev = 0;

        `uvm_info("OHSEQ", $sformatf("fgOTN %s: %0d 平面，每平面总时隙 %0d（随机拆通道，每平面 ≤%0d 通道）",
        (dir == 0) ? "OH_EXT" : "OH_INS", n_pl, cfg.oh_slots_total, max_ch), UVM_LOW)

        for (int p = 0; p < n_pl; p++) begin
            n_ch = 1 + ($urandom % max_ch); // 随机通道数 1..max_ch
            gen_slots(cfg.oh_slots_total, n_ch, slots_arr);
            for (int ch = 0; ch < n_ch; ch++) begin
                real tfg = 122368.0 / (slots_arr[ch] * 10.409203e6);
                real ch_skew = ch * tfg / n_ch; // 平面内通道帧头错开
                for (int N = 0; ; N++) begin
                    int added = 0;
                    for (int k = 0; k < 8; k++) begin
                        t = p * skew + ch_skew + (N * 15296 + oh_off(k)) * tfg / 15296.0;
                        if (t >= run_s) break;
                        t_arr = new[t_arr.size()+1](t_arr);
                        plane_arr = new[plane_arr.size()+1](plane_arr);
                        chan_arr = new[chan_arr.size()+1](chan_arr);
                        nfg_arr = new[nfg_arr.size()+1](nfg_arr);
                        k_arr = new[k_arr.size()+1](k_arr);
                        pri_arr = new[pri_arr.size()+1](pri_arr);
                        t_arr[t_arr.size()-1] = t;
                        plane_arr[plane_arr.size()-1] = p;
                        chan_arr[chan_arr.size()-1] = ch;
                        nfg_arr[nfg_arr.size()-1] = N;
                        k_arr[k_arr.size()-1] = k;
                        pri_arr[pri_arr.size()-1] =
                            (dir == 0) ? 7 : ((slots_arr[ch] <= 20) ? 1 : 0);
                        added++;
                    end
                    if (added == 0) break;
                end
            end
        end
        n_ev = t_arr.size();

        for (int i = 1; i < n_ev; i++) begin
            real tv = t_arr[i];
            int pv = plane_arr[i], cv = chan_arr[i], nv = nfg_arr[i], kv = k_arr[i];
            int pv2 = pri_arr[i];
            int j = i - 1;
            while (j >= 0 && t_arr[j] > tv) begin
                t_arr[j+1] = t_arr[j];
                plane_arr[j+1] = plane_arr[j];
                chan_arr[j+1] = chan_arr[j];
                nfg_arr[j+1] = nfg_arr[j];
                k_arr[j+1] = k_arr[j];
                pri_arr[j+1] = pri_arr[j];
                j--;
            end
            t_arr[j+1] = tv;
            plane_arr[j+1] = pv;
            chan_arr[j+1] = cv;
            nfg_arr[j+1] = nv;
            k_arr[j+1] = kv;
            pri_arr[j+1] = pv2;
        end
        `uvm_info("OHSEQ", $sformatf("共 %0d 条 fgOTN KO", n_ev), UVM_LOW)

        for (int i = 0; i < n_ev; i++) begin
            real t_clks = t_arr[i] * cfg.clk_freq_hz;
            if (t_clks < cursor + 1.0) t_clks = cursor + 1.0;
            while (cursor < t_clks) begin
                @(posedge cfg.vif.clk);
                cursor += 1.0;
            end
            begin
                koa_item it = koa_item::type_id::create("it");
                it.pri = pri_arr[i];
                it.stream = (dir == 0) ? ST_OH_EXT : ST_OH_INS;
                it.plane = plane_arr[i];
                it.cid = plane_arr[i] * max_ch + chan_arr[i];
                it.pos = k_arr[i];
                start_item(it);
                finish_item(it);
            end
        end
    endtask
endclass
