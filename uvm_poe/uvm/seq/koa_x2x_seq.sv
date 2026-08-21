// X2X 源序列（APS_EXT / APS_INS / ALM）：8 平面，每平面随机通道时隙表
// APS 每帧 1 位置 = 1 条 KO；ALM 每帧 4 位置 = 4 条 KO；优先级全部按带宽规则
class koa_x2x_seq extends uvm_sequence #(koa_item);
    `uvm_object_utils(koa_x2x_seq)
    int kind; // 0=APS_EXT，1=APS_INS，2=ALM

    function new(string name = "koa_x2x_seq");
        super.new(name);
        kind = 0;
    endfunction

    function int alm_off(int k);
        if (k > 3) k = 3;
        return k * 3824;
    endfunction

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
        int n_pl, max_ch, ko_per_frame;
        int n_ch;
        int slots_arr[];
        real t_arr[];
        int plane_arr[], chan_arr[], nfg_arr[], k_arr[];
        int pri_arr[];
        int n_ev;

        cfg = ko_pkg::g_tb_cfg;
        while (!ko_pkg::g_reset_done) @(posedge cfg.vif.clk);

        n_pl = (cfg.n_x2x_planes > 0) ? cfg.n_x2x_planes : 1;
        max_ch = (cfg.n_ch_per_plane > 0) ? cfg.n_ch_per_plane : 1;
        run_s = cfg.run_us * 1e-6;
        cursor = 0.0;
        n_ev = 0;
        ko_per_frame = (kind == 2) ? 4 : 1;

        `uvm_info("XSEQ", $sformatf("X2X %s: %0d 平面，每平面总时隙 %0d（随机拆通道，≤%0d 通道），%0d KO/帧",
        (kind == 0) ? "APS_EXT" : ((kind == 1) ? "APS_INS" : "ALM"),
        n_pl, cfg.x2x_slots_total, max_ch, ko_per_frame), UVM_LOW)

        for (int p = 0; p < n_pl; p++) begin
            n_ch = 1 + ($urandom % max_ch);
            gen_slots(cfg.x2x_slots_total, n_ch, slots_arr);
            for (int ch = 0; ch < n_ch; ch++) begin
                real tfg = 122368.0 / (slots_arr[ch] * 10.409203e6);
                real ch_skew = ch * tfg / n_ch;
                for (int N = 0; ; N++) begin
                    int added = 0;
                    for (int k = 0; k < ko_per_frame; k++) begin
                        int off = (kind == 2) ? alm_off(k) : 0;
                        t = p * skew + ch_skew + (N * 15296 + off) * tfg / 15296.0;
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
                        pri_arr[pri_arr.size()-1] = (slots_arr[ch] <= 20) ? 1 : 0;
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
        `uvm_info("XSEQ", $sformatf("共 %0d 条 X2X KO", n_ev), UVM_LOW)

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
                case (kind)
                    0: it.stream = ST_APS_EXT;
                    1: it.stream = ST_APS_INS;
                    default: it.stream = ST_ALM;
                endcase
                it.plane = plane_arr[i];
                it.cid = plane_arr[i] * max_ch + chan_arr[i];
                it.pos = k_arr[i];
                start_item(it);
                finish_item(it);
            end
        end
    endtask
endclass
