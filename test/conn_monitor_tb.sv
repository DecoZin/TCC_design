// ---------------------------------------------------------
// Module: BLE Setup Controller Module testbench
// Description: Testbench for the BLE Setup Controller
// Author: André Lamego
// Date: 2025-10-17
// ---------------------------------------------------------
`timescale 1ns / 100ps

module conn_monitor_tb;
    import tasks_pkg::*;	
    import conn_monitor_types_pkg::*;
    import ei_mem_pkg::*;

    // Local Parameters
    localparam int CLK_FREQ_HZ   = 50_000_000;
    localparam int CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ_HZ;
    localparam int CMD_WIDTH = 16;
    localparam int CMD_DEPTH = 3;
    localparam int DATA_WIDTH = 8;
    localparam int DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;
    localparam real BAUD_RATE = 9600; // Baud rate
    localparam int BIT_PERIOD = $rtoi(1e9 / BAUD_RATE); // = 104_166.66 ns (when timescale is 1ns)

    // SIMULATION TIMEOUT
    localparam int SIMULATION_TIMEOUT = 800*BIT_PERIOD;

    // Clock and Reset
    logic clk;
    logic rst_n = 1'b1;

    // Error flag
    logic test_fail = 1'b0;

    // Interfaces instantiation
    regs_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_spec_regs(.clk(clk), .rst_n(rst_n));

    regs_int_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link(.clk(clk), .rst_n(rst_n));


    tmr_if if_conn_monitor_tmr(.clk(clk), .rst_n(rst_n));;

    // Connection Monitor module Instantiation
    logic timeout;
    logic connect;
    logic disconnect;
    logic setup_done = 1'b0;

    connection_monitor dut (
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
        .get_ack_byte(rd_en_ble),
        // Signals
        .setup_done(setup_done),
        .connect(connect),
        .disconnect(disconnect),
        .timeout(timeout),
        // Timer parameters
        .reg_adv_time_count(reg_adv_time_count),
        .reg_conn_time_count(reg_conn_time_count)
    );

    // Timer Instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) conn_monitor_timer (
        .if_t(if_conn_monitor_tmr.timer),
        .clk(clk),
        .rst_n(rst_n)
    );
        
    // "Special Registers" Instantiation (Just a Register File with some default values)
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(DATA_DEPTH)
    ) spec_regs (
        .if_regs_inst(if_spec_regs.slave),
        .if_top_link(if_top_link.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    logic [23:0] reg_adv_time_count;
    logic [23:0] reg_conn_time_count;
    
    always_comb begin
        if_top_link.mode_mask = '0; // All registers are Read-Write
        if_top_link.regi[EIR_ADV_TMR0] = 8'(20*BIT_PERIOD/1000);
        if_top_link.regi[EIR_ADV_TMR1] = 8'((20*BIT_PERIOD/1000) >> 8);
        if_top_link.regi[EIR_ADV_TMR2] = 8'((20*BIT_PERIOD/1000) >> 16);
        if_top_link.regi[EIR_CONN_TMR0] = 8'(15*BIT_PERIOD/1000);
        if_top_link.regi[EIR_CONN_TMR1] = 8'((15*BIT_PERIOD/1000) >> 8);
        if_top_link.regi[EIR_CONN_TMR2] = 8'((15*BIT_PERIOD/1000) >> 16);
        reg_adv_time_count = {if_top_link.rego[EIR_ADV_TMR2],
                              if_top_link.rego[EIR_ADV_TMR1],
                              if_top_link.rego[EIR_ADV_TMR0]};
        reg_conn_time_count = {if_top_link.rego[EIR_CONN_TMR2],
                               if_top_link.rego[EIR_CONN_TMR1],
                               if_top_link.rego[EIR_CONN_TMR0]};
    end

    // UART Instantiation
    logic baud = 1'b0;            // Signal toggling at baud rate
    logic ble_rx_pin = 1'b1;
    wire rd_en_ble;
    wire [7:0] rx_sys_data;
    wire valid_setup = 1'b0;
    wire [7:0] tx_setup = '0;
    logic ble_tx_pin;
    wire valid_rx;
    wire rx_fifo_ready;
    wire tx_full;
    wire tx_done;

    assign rx_fifo_rd_en = (dut.state == S_CONNECTED) ? rd_en_processor : rd_en_ble;

    logic [2:0] uart_rx_state; // (Not used)
    logic [2:0] uart_tx_state; // (Not used)
    logic [1:0] rx_fifo_fe;    // (Not used)
    logic [1:0] tx_fifo_fe;    // (Not used)

    UART #(.BAUD(BAUD_RATE), .CLK_F(CLK_FREQ_HZ)) uart (
        .clk(clk),
        .rst_n(rst_n),
        
        .i_test_mux(1'b0),  // 0: normal | 1: test
        .i_transmit(1'b0),  // Transmit data from RX to TX (not used)
        .i_rx_serial_ext(ble_rx_pin),
        .i_rx_fifo_rd_en(rx_fifo_rd_en),
        .i_valid_tx(valid_setup),
        .i_tx_sys_data(tx_setup),
        
        .o_rx_sys_data(rx_sys_data),
        .o_valid_rx(valid_rx),
        .o_rx_ready(rx_fifo_ready),
        .o_tx_serial_ext(ble_tx_pin),
        .o_tx_full(tx_full),
        .o_done(tx_done),

        .uart_rx_state(uart_rx_state), // (Not used)
        .uart_tx_state(uart_tx_state), // (Not used)
        .rx_fifo_fe(rx_fifo_fe),       // (Not used)
        .tx_fifo_fe(tx_fifo_fe)        // (Not used)
    );
        
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

    // Monitor for Connection Monitor Timer
    always_ff @(posedge clk) begin
        if (if_conn_monitor_tmr.done) begin
            if (dut.state == S_CONNECTED) begin
                DEBUG_WARN("DUT", "Connection Timeout Hit!");
            end else begin
                DEBUG_WARN("DUT", "Advertisement Timeout Hit!");
            end
        end
    end

    // Monitor of Timeout flag
    always_ff @(posedge clk) begin
        if (timeout) begin
            DEBUG_WARN("DUT", "Timeout Reported by DUT!");
        end
    end

    // FSM Transitions Monitor
    conn_monitor_state_t prev_state;
    logic illegal_transition;
    
    always_comb begin
        illegal_transition = 1'b0;
        if (prev_state != dut.state) begin
            if (prev_state == S_IDLE && 
                !(dut.state inside {S_WAIT_EVENT})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == S_WAIT_EVENT && 
                !(dut.state inside {S_IDLE, S_PARSE_MAC})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == S_PARSE_MAC && 
                !(dut.state inside {S_STORE_MAC})) begin
                    illegal_transition = 1'b1;
            end
            if (prev_state == S_STORE_MAC && 
                !(dut.state inside {S_CONNECTED})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == S_CONNECTED &&
                !(dut.state inside {S_HANDLE_DISCONNECT})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == S_HANDLE_DISCONNECT &&
                !(dut.state inside {S_IDLE})) begin    
                illegal_transition = 1'b1;
            end
        end    
    end

    always_ff @(posedge clk) begin
        prev_state <= dut.state;
        if (!rst_n) begin
            DEBUG_INFO("DUT", "FSM: IDLE"); 
        end else begin
            if (dut.state != prev_state) begin
                if (illegal_transition) begin
                    DEBUG_ERR("DUT", $sformatf("Illegal state transition: %s -> %s", prev_state.name(), dut.state.name()), test_fail);
                end else begin
                    DEBUG_INFO("DUT", $sformatf("FSM: %s -> %s", prev_state.name(), dut.state.name()));
                end
            end        
        end
    end

    // UART TX Monitor
    int uart_tx_index;
    byte received;

    always @(negedge ble_tx_pin) begin
        #(BIT_PERIOD + CLK_PERIOD_NS);

        for (uart_tx_index = 0; uart_tx_index < 8; uart_tx_index++) begin
            received[uart_tx_index] <= ble_tx_pin;
            #(BIT_PERIOD);
        end

        if (ble_tx_pin !== 1) begin
            DEBUG_ERR("UART", "UART ERROR: Stop bit not high", test_fail);
        end else begin
            if (received == 8'h0D) begin
                DEBUG_INFO("UART", "UART Received: \\r");
            end else if (received == 8'h0A) begin
                DEBUG_INFO("UART", "UART Received: \\n");
            end else begin
                DEBUG_INFO("UART", $sformatf("UART Received: %s", received));
            end
        end
    end

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

    // Register Write Monitor
    always @(posedge clk) begin
        if (if_spec_regs.write_en) begin
            DEBUG_INFO("DUT", $sformatf("Register Write: ADDR: 0x%x, DATA: 0x%x", if_spec_regs.addr, if_spec_regs.write_data));
        end
    end

    // Simulation of other module reading UART
    logic rd_en_processor;
    assign rd_en_processor = valid_rx;

    // Stimulus
    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        if_top_link.load_regs = 1'b0;
        setup_done = 1'b1;
        #(CLK_PERIOD_NS);
        reset_n(rst_n, CLK_PERIOD_NS);
        #(2*CLK_PERIOD_NS); 
        
        DEBUG_INFO("TB", "Testing Connection...");
        DEBUG_INFO("TB", "Sending: \"OK+CONN:010203040506\\r\\n...");
        send_uart_string(ble_rx_pin, $sformatf("OK+CONN:010203040506%c\n", 8'h0D), BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Connection... Done.");

        #(BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Disconnection...");
        DEBUG_INFO("TB", "Sending: \"OK+DISC\\r\\n...");
        send_uart_string(ble_rx_pin, $sformatf("OK+DISC%c\n", 8'h0D), BIT_PERIOD);
        DEBUG_INFO("TB", "Testing Disconnection... Done.");

        #(CLK_PERIOD_NS);
        DEBUG_INFO("TB", "Testing Timeout...");
        @(if_conn_monitor_tmr.done);
        send_uart_string(ble_rx_pin, $sformatf("OK+CONN:010203040506%c\n", 8'h0D), BIT_PERIOD);
        @(if_conn_monitor_tmr.done);
        DEBUG_INFO("TB", "Testing Timeout... Done.");

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
        $dumpvars(0, conn_monitor_tb); // Dump all signals in the testbench
    end

endmodule
