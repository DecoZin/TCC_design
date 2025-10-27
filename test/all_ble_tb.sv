// ---------------------------------------------------------
// Module: All BLE modules testbench
// Description: Testbench for the All BLE modules
// Author: André Lamego
// Date: 2025-10-26
// ---------------------------------------------------------
`timescale 1ns / 100ps

module all_ble_tb;
    import tasks_pkg::*;	
    import ei_mem_pkg::*;
    import cmd_mem_pkg::*;
    import ble_ctrl_types_pkg::*;
    import ble_setup_types_pkg::*;
    import conn_monitor_types_pkg::*;

    // Local Parameters
    localparam int CLK_FREQ_HZ    = 50_000_000;
    localparam int CLK_PERIOD_NS  = 1_000_000_000 / CLK_FREQ_HZ;
    localparam int DATA_WIDTH     = 8;
    localparam int CMD_WIDTH      = 32;
    localparam int CMD_DEPTH = 16;
    localparam int CMD_DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;
    localparam int SR_DATA_DEPTH  = 43;
    localparam int BUS_DATA_DEPTH = 1024;
    localparam real BAUD_RATE     = 9600; // Baud rate
    localparam int BIT_PERIOD     = $rtoi(1e9 / BAUD_RATE); // = 104_166.66 ns (when timescale is 1ns)
    localparam int BYTE_PERIOD    = 10*BIT_PERIOD;
    localparam int TIMEOUT_US     = $rtoi(3*BYTE_PERIOD/1000);
    localparam int DS_TIMEOUT_US  = $rtoi(BIT_PERIOD/1000);

    // SIMULATION TIMEOUT
    localparam int SIMULATION_TIMEOUT = 800*BYTE_PERIOD;

    // Clock and Reset
    logic clk;
    logic rst_n = 1'b1;

    // Error flag
    logic test_fail = 1'b0;

    // Interfaces instantiation
    regs_if #(
        .DATA_DEPTH(SR_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_spec_regs(.clk(clk), .rst_n(rst_n));

    regs_if #(
        .DATA_DEPTH(CMD_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_cmd_mem(.clk(clk), .rst_n(rst_n));

    regs_int_if #(
        .DATA_DEPTH(SR_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link(.clk(clk), .rst_n(rst_n));

    tmr_if if_ds_timer(.clk(clk), .rst_n(rst_n));
    tmr_if if_ack_timer(.clk(clk), .rst_n(rst_n));;
    tmr_if if_conn_monitor_tmr(.clk(clk), .rst_n(rst_n));;

    // UART Instantiation
    logic baud = 1'b0;            // Signal toggling at baud rate
    logic rx_serial = 1'b1;
    logic tx_serial;
    wire [7:0] rx_sys_data;
    wire rd_fifo_rd_en;
    wire rx_fifo_ready;
    wire valid_rx;
    wire [7:0] tx_sys_data;
    wire valid_tx;
    wire tx_full;
    wire tx_done;

    logic [2:0] uart_rx_state; // (Not used)
    logic [2:0] uart_tx_state; // (Not used)
    logic [1:0] rx_fifo_fe;    // (Not used)
    logic [1:0] tx_fifo_fe;    // (Not used)

    UART #(.BAUD(BAUD_RATE), .CLK_F(CLK_FREQ_HZ)) uart (
        .clk(clk),
        .rst_n(rst_n),
        
        .i_test_mux(1'b0),  // 0: normal | 1: test
        .i_transmit(1'b1),  // Transmit data from RX to TX (not used)
        
        .i_rx_serial_ext(rx_serial),
        .i_rx_fifo_rd_en(rd_fifo_rd_en),
        .i_tx_sys_data(tx_sys_data),
        .i_valid_tx(valid_tx),
        
        .o_tx_serial_ext(tx_serial),
        .o_rx_sys_data(rx_sys_data),
        .o_rx_ready(rx_fifo_ready),
        .o_valid_rx(valid_rx),
        .o_tx_full(tx_full),
        .o_done(tx_done),

        .uart_rx_state(uart_rx_state), // (Not used)
        .uart_tx_state(uart_tx_state), // (Not used)
        .rx_fifo_fe(rx_fifo_fe),       // (Not used)
        .tx_fifo_fe(tx_fifo_fe)        // (Not used)
    );

    // UART Mux
    logic [7:0] m_tx_data;
    logic m_valid_tx;
    wire sel_tx_setup;
    wire [7:0] tx_controller_data;
    wire tx_controller_valid;
    wire [7:0] tx_setup_data;
    wire tx_setup_valid;

    assign tx_sys_data = m_tx_data;
    assign valid_tx = m_valid_tx;
    assign m_tx_data  = sel_tx_setup ? tx_controller_data : tx_setup_data;
    assign m_valid_tx = sel_tx_setup ? tx_controller_valid : tx_setup_valid;

    logic m_rd_en_setup;
    logic rd_en_processor;
    wire [1:0] sel_rx_setup;
    wire rd_en_conn;
    wire rd_en_ble;
    wire rd_en_cmd;

    assign rd_fifo_rd_en = mux_transceiver ? rd_en_processor : m_rd_en_setup;
    always_comb begin : rx_setup_mux
        case (sel_rx_setup)
            0: m_rd_en_setup = rd_en_ble;
            1: m_rd_en_setup = rd_en_cmd;
            2: m_rd_en_setup = rd_en_conn; 
            4: m_rd_en_setup = 1'b0;
        endcase
    end
    // Simulation of other module reading UART
    assign rd_en_processor = valid_rx;

    // BLE Setup Controller module instantiation
    wire en_cmd_mem_wr;
    wire setting_up;
    logic mux_transceiver; // (Not used)

    ble_setup_controller controller (
        // Interfaces
        .if_ds_timer(if_ds_timer.controller),
        .regs_slp_time_count(regs_slp_time_count),       
        // Clock and Reset
        .clk(clk),
        .rst_n(rst_n),
        // BLE Setup Interface
        .setting_up(setting_up),
        .setup_done(setup_done),
        .fail(fail),
        // Connection Monitor Interface
        .disconnect(disconnect),
        .connect(connect),
        .time_out(timeout),
        // External Signals
        .programming(1'b0),
        .direct_conn(1'b0),
        // Command Memory Interface
        .en_cmd_mem_wr(en_cmd_mem_wr),
        .error_pulse(error_pulse),
        .error_code(error_code),
        // UART TX Interface
        .tx_valid(tx_controller_valid),
        .tx_data(tx_controller_data),
        .tx_full(tx_full),
        // Muxes
        .mux_rx_setup(sel_rx_setup),
        .mux_tx_setup(sel_tx_setup),
        .mux_transceiver(mux_transceiver)
    );

    // Deep Sleep Timer Module instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) timer_deep_sleep (
        .if_t(if_ds_timer.timer),
        .clk(clk),
        .rst_n(rst_n)
    );

    // BLE Setup module Instantiation
    wire fail;
    wire setup_done;

    ble_setup #(
        .CMD_WIDTH(CMD_WIDTH)
    ) setup (
            // Interfaces
        .if_regs_inst(if_cmd_mem.master),
        .if_tmr_inst(if_ack_timer.controller),
        .regs_ack_time_count(regs_ack_time_count),
        // Clock and Reset
        .clk(clk),
        .rst_n(rst_n),
        // UART RX
        .ack_valid(valid_rx),
        .ack_ready(rx_fifo_ready),
        .ack_byte(rx_sys_data),
        .get_ack_byte(rd_en_ble),
        // Signals
        .setting_up(setting_up),
        .fail(fail),
        .setup_done(setup_done),
        // UART TX
        .tx_done(tx_done),
        .tx_full(tx_full),
        .byte_ready(tx_setup_valid),
        .cmd_byte(tx_setup_data)
    );

    // Acknowledgement Timer Instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) ack_timer (
        .if_t(if_ack_timer.timer),
        .clk(clk),
        .rst_n(rst_n)
    );

    // Command Memory Instatiation
    wire [1:0] error_code;
    wire error_pulse;
    
    cmd_memory #(
        .CMD_WIDTH(CMD_WIDTH),   // Number of bytes per command
        .CMD_DEPTH(CMD_DEPTH)    // Number of commands in memory
    ) cmds (
        .if_regs_inst(if_cmd_mem.slave),
        
        .clk(clk),
        .rst_n(rst_n),
        
        .cmd_data(rx_sys_data),
        .rd_en(rd_en_cmd),
        .data_ready(rx_fifo_ready),
        .data_valid(valid_rx),
        
        .enable(en_cmd_mem_wr),
        .error_code(error_code),
        .error_pulse(error_pulse)
    );

    // Connection Monitor module Instantiation
    wire timeout;
    wire connect;
    wire disconnect;

    connection_monitor monitor (
        // Interfaces
        .if_regs_inst(if_spec_regs.master),
        .if_tmr(if_conn_monitor_tmr.controller),
        // Clock and Reset
        .clk(clk),
        .rst_n(rst_n),
        // UART RX
        .ack_byte(rx_sys_data),
        .ack_valid(valid_rx),
        .ack_ready(rx_fifo_ready),
        .get_ack_byte(rd_en_conn),
        // Signals
        .setup_done(setup_done),
        .connect(connect),
        .disconnect(disconnect),
        .timeout(timeout),
        // Timer parameters
        .regs_adv_time_count(regs_adv_time_count),
        .regs_conn_time_count(regs_conn_time_count)
    );

    // Connection Monitor Timer Instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) conn_monitor_timer (
        .if_t(if_conn_monitor_tmr.timer),
        .clk(clk),
        .rst_n(rst_n)
    );

    // "Special Registers" Instantiation (Just a Register File with some default values)
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(SR_DATA_DEPTH)
    ) spec_regs (
        .if_regs_inst(if_spec_regs.slave),
        .if_top_link(if_top_link.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    logic [23:0] regs_ack_time_count;
    logic [23:0] regs_adv_time_count;
    logic [23:0] regs_conn_time_count;
    logic [23:0] regs_delay_time_count;
    logic [23:0] regs_opds_time_count;
    logic [23:0] regs_slp_time_count;
    
    always_comb begin
        if_top_link.mode_mask = '0; // All special registers are Read-Write
        if_top_link.load_regs = '0; // We won't load any special registers        
        if_top_link.regi = '{default: '0}; // Default values for all special registers

        if_top_link.regi[EIR_ACK_TMR0] = 8'(TIMEOUT_US);
        if_top_link.regi[EIR_ACK_TMR1] = 8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_ACK_TMR2] = 8'(TIMEOUT_US >> 16);

        if_top_link.regi[EIR_ADV_TMR0] = 8'(TIMEOUT_US);
        if_top_link.regi[EIR_ADV_TMR1] = 8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_ADV_TMR2] = 8'(TIMEOUT_US >> 16);
        
        if_top_link.regi[EIR_CONN_TMR0] = 8'(TIMEOUT_US);
        if_top_link.regi[EIR_CONN_TMR1] = 8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_CONN_TMR2] = 8'(TIMEOUT_US >> 16);
        
        if_top_link.regi[EIR_DELAY_TMR0] = 8'(TIMEOUT_US);
        if_top_link.regi[EIR_DELAY_TMR1] = 8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_DELAY_TMR2] = 8'(TIMEOUT_US >> 16);
        
        if_top_link.regi[EIR_OPDS_TMR0] =  8'(TIMEOUT_US);
        if_top_link.regi[EIR_OPDS_TMR1] =  8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_OPDS_TMR2] =  8'(TIMEOUT_US >> 16);

        if_top_link.regi[EIR_SLP_TMR0] = 8'(DS_TIMEOUT_US);
        if_top_link.regi[EIR_SLP_TMR1] = 8'(DS_TIMEOUT_US >> 8);
        if_top_link.regi[EIR_SLP_TMR2] = 8'(DS_TIMEOUT_US >> 16);

        regs_ack_time_count   = {if_top_link.rego[EIR_ACK_TMR2],
                                 if_top_link.rego[EIR_ACK_TMR1],
                                 if_top_link.rego[EIR_ACK_TMR0]};
        regs_adv_time_count   = {if_top_link.rego[EIR_ADV_TMR2],
                                 if_top_link.rego[EIR_ADV_TMR1],
                                 if_top_link.rego[EIR_ADV_TMR0]};
        regs_conn_time_count  = {if_top_link.rego[EIR_CONN_TMR2],
                                 if_top_link.rego[EIR_CONN_TMR1],
                                 if_top_link.rego[EIR_CONN_TMR0]};
        regs_delay_time_count = {if_top_link.rego[EIR_DELAY_TMR2],
                                 if_top_link.rego[EIR_DELAY_TMR1],
                                 if_top_link.rego[EIR_DELAY_TMR0]};
        regs_opds_time_count  = {if_top_link.rego[EIR_OPDS_TMR2],
                                 if_top_link.rego[EIR_OPDS_TMR1],
                                 if_top_link.rego[EIR_OPDS_TMR0]};
        regs_slp_time_count   = {if_top_link.rego[EIR_SLP_TMR2],
                                 if_top_link.rego[EIR_SLP_TMR1],
                                 if_top_link.rego[EIR_SLP_TMR0]};
    end

    // Clock Generation
    initial fork
        generate_clock(clk, {{32{$rtoi(CLK_PERIOD_NS / 2)[31]}},$rtoi(CLK_PERIOD_NS / 2)});
    join_none

    initial fork
        generate_clock(baud, {{32{BIT_PERIOD[31]}},BIT_PERIOD});
    join_none


    // Global timeout for stuck simulation
    initial begin
        #SIMULATION_TIMEOUT;
        DEBUG_ERR("TB", "Global timeout hit! Simulation stuck.", test_fail);
        DEBUG_FAIL();
        $finish;
    end

    // UART TX Monitor
    int uart_tx_index;
    byte received;
    bit lf_flag;

    always @(negedge tx_serial) begin
        #(BIT_PERIOD + CLK_PERIOD_NS);
        
        for (uart_tx_index = 0; uart_tx_index < 8; uart_tx_index++) begin
            received[uart_tx_index] <= tx_serial;
            #(BIT_PERIOD);
        end

        if (tx_serial !== 1) begin
            DEBUG_ERR("UART", "UART ERROR: Stop bit not high", test_fail);
        end else begin
            if (received == 8'h0D) begin
                DEBUG_INFO("UART", "UART Received: \\r");
            end else if (received == 8'h0A) begin
                DEBUG_INFO("UART", "UART Received: \\n");
            end else if ((received >= "a" && received <= "z") ||
                         (received >= "A" && received <= "Z") || 
                         (received >= "0" && received <= "9") ||
                         (received == ",") || (received == "!") ||
                         (received == "+") || (received == ":")) begin
                DEBUG_INFO("UART", $sformatf("UART Received: %s", received));
            end else begin
                DEBUG_INFO("UART", $sformatf("UART Received: 0x%x", received));
            end
        end
    end

    assign lf_flag = (received == 8'h0A);

    // Connection & Disconnection Monitor
    always @(posedge connect or posedge disconnect) begin
        if (connect && disconnect) begin
            DEBUG_ERR("DUT", "BLE Connected and Disconnected at the same time!", test_fail);
        end else if (connect) begin
            DEBUG_INFO("DUT", "BLE Connected!");
        end else if (disconnect) begin
            DEBUG_WARN("DUT", "BLE Disconnected!");
        end
    end

    // Stimulus
    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        #(CLK_PERIOD_NS);
        reset_n(rst_n, CLK_PERIOD_NS);
        #(CLK_PERIOD_NS);
        
        DEBUG_INFO("TB", "Testing BLE setup...");
        @(setup.state == SEND_CMD);
        for (int i = 0; i < setup.cmd_number; i++) begin
            @(posedge lf_flag);
            #(BYTE_PERIOD);
            DEBUG_INFO("TB", "Simulating BLE response...");
            send_uart_string(rx_serial, $sformatf("OK%c\n", 8'h0D), BIT_PERIOD);
            DEBUG_INFO("TB", "Simulating BLE response... Done.");
        end
        @(posedge setup_done);
        DEBUG_INFO("DUT", "BLE setup done!");
        DEBUG_INFO("TB", "Testing BLE setup... Done.");

        DEBUG_INFO("TB", "Testing Connection...");
        DEBUG_INFO("TB", "Sending: \"OK+CONN:010203040506\\r\\n...");
        send_uart_string(rx_serial, $sformatf("OK+CONN:010203040506%c\n", 8'h0D), BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Connection... Done.");

        #(BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Disconnection...");
        DEBUG_INFO("TB", "Sending: \"OK+DISC\\r\\n...");
        send_uart_string(rx_serial, $sformatf("OK+DISC%c\n", 8'h0D), BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Disconnection... Done.");

        #(5*CLK_PERIOD_NS);
        DEBUG_INFO("TB", "Testbench finished.");
        #(CLK_PERIOD_NS);
        if (test_fail) begin
            DEBUG_FAIL();
        end else begin
            DEBUG_PASS();
        end
        #1;
        $finish;
    end

    initial begin
        $dumpfile("waveform.fst");  // Specify the waveform file name
        $dumpvars(0, processor_tb); // Dump all signals in the testbench
    end

endmodule
