// 7 条 KO 输入流
typedef enum {
    ST_OH_EXT, // fgOTN 开销提取（pri=7）
    ST_OH_INS, // fgOTN 开销下插（pri=带宽）
    ST_APS_EXT, // X2X APS 提取（pri=带宽）
    ST_APS_INS, // X2X APS 下插（pri=带宽）
    ST_ALM, // X2X ALM（pri=带宽）
    ST_UART_EXT, // 串口提取（pri=7）
    ST_UART_INS // 串口下插（pri=随机）
} koa_stream_t;

// KOA 输入/输出事务：有效信息（3bit 调度优先级 + 流/平面/通道/位置 + 预读）。
// 注：链路不再承载 48B KO 报文——KOA 只缓存/调度有效信息，线程描述由 THM
// 从线程模板池随机选取（见 poe_thread_tpl.sv）。
class koa_item extends uvm_sequence_item;
    rand bit [2:0] pri; // 调度优先级（写入报文行1 PRI）
    koa_stream_t stream; // 输入流（输出事件由 out_src 映射）
    int plane; // 平面号（串口恒 0）
    bit [16:0] cid; // 通道号（1 时隙粒度；串口恒 0）
    bit [2:0] pos; // 该流内开销位置序号（串口恒 0）
    int sbuf; // SBUF 编号：0=EXT 1=INS 2=ALM 3=UART_EXT 4=UART_INS
    longint ev_time; // 事件上报时间戳（check_phase 排序用）
    bit is_out; // 1=输出事件，0=输入事件

    `uvm_object_utils_begin(koa_item)
    `uvm_field_int(pri, UVM_ALL_ON)
    `uvm_field_enum(koa_stream_t, stream, UVM_ALL_ON)
    `uvm_field_int(cid, UVM_ALL_ON)
    `uvm_field_int(pos, UVM_ALL_ON)
    `uvm_field_int(sbuf, UVM_ALL_ON)
    `uvm_field_int(plane, UVM_ALL_ON)
    `uvm_field_int(ev_time, UVM_ALL_ON)
    `uvm_field_int(is_out, UVM_ALL_ON)
    `uvm_object_utils_end

    function new(string name = "koa_item");
        super.new(name);
    endfunction
endclass
