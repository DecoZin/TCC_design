// ---------------------------------------------------------
// Module: BLE Setup Controller Module testbench
// Description: Testbench for the BLE Setup Controller
// Author: André Lamego
// Date: 2025-10-17
// ---------------------------------------------------------
`timescale 1ns / 100ps

module ble_setup_tb;
    import tasks_pkg::*;	
    import ble_setup_types_pkg::*;
    import cmd_mem_pkg::*;

    // Local Parameters
    localparam int CLK_FREQ_HZ   = 50_000_000;
    localparam int CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ_HZ;
    localparam int CMD_WIDTH = 16;
    localparam int CMD_DEPTH = 3;
    localparam int DATA_WIDTH = 8;
    localparam int DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;
    localparam real BAUD_RATE = 9600; // Baud rate
    localparam int BIT_PERIOD = $rtoi(1e9 / BAUD_RATE); // = 104_166.66 ns (when timescale is 1ns)
    localparam logic [23:0] regs_ack_time_count = 24'd2_100; // 20 bits at 9600 baud

    // SIMULATION TIMEOUT
    localparam int SIMULATION_TIMEOUT = 500*BIT_PERIOD;

    // Clock and Reset
    logic clk;
    logic rst_n = 1'b1;

    // Error flag
    logic test_fail = 1'b0;

    // Interfaces instantiation
    regs_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_regs_inst(.clk(clk), .rst_n(rst_n));

    regs_int_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link(.clk(clk), .rst_n(rst_n));

    tmr_if if_tmr_inst(.clk(clk), .rst_n(rst_n));;

    // BLE Setup module Instantiation
    logic setting_up = 1'b0;
    logic fail;
    logic setup_done;

    ble_setup #(
        .CMD_WIDTH(CMD_WIDTH)
        ) dut (
            // Interfaces
        .if_regs_inst(if_regs_inst.master),
        .if_tmr_inst(if_tmr_inst.controller),
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
        .byte_ready(valid_setup),
        .cmd_byte(tx_setup)
    );

    // Timer Instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) ack_timer (
        .if_t(if_tmr_inst.timer),
        .clk(clk),
        .rst_n(rst_n)
    );
        
    // "Comand Memory" Instantiation (Just a Register File with some default values)
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(DATA_DEPTH)
    ) cmd_memory (
        .if_regs_inst(if_regs_inst.slave),
        .if_top_link(if_top_link.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    logic test_zero_cmd = 1'b0;

    always_comb begin
        if_top_link.mode_mask = '1; // All registers are Read-Only
        if_top_link.regi[0] = (!test_zero_cmd) ? 8'h00 : 8'h02; // Number of commands
        assign_string_to_regi(HM10_CMD_AT,      1);
        assign_string_to_regi(HM10_CMD_NAME,   17);
    end

    function void assign_string_to_regi(input string str, input int start_idx);
        for (int i = 0; i < str.len(); i++) begin
            if_top_link.regi[start_idx + i] = str[i];
        end
    endfunction

    // UART Instantiation
    logic baud = 1'b0;            // Signal toggling at baud rate
    logic ble_rx_pin = 1'b1;
    wire rd_en_ble;
    wire [7:0] rx_sys_data;
    wire valid_setup;
    wire [7:0] tx_setup;
    logic ble_tx_pin;
    wire valid_rx;
    wire rx_fifo_ready;
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
        .i_rx_serial_ext(ble_rx_pin),
        .i_rx_fifo_rd_en(rd_en_ble),
        .i_valid_tx(valid_setup),
        .i_tx_sys_data(tx_setup),
        
        .o_tx_serial_ext(ble_tx_pin),
        .o_rx_sys_data(rx_sys_data),
        .o_valid_rx(valid_rx),
        .o_done(tx_done),
        .o_rx_ready(rx_fifo_ready),
        .o_tx_full(tx_full),

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

    // Monitor for Acknowledgement Timer
    always_ff @(posedge clk) begin
        if (if_tmr_inst.done) begin
            DEBUG_WARN("DUT", "Acknowledgement Timeout Hit!");
        end
    end

    // Monitor of Failure
    always_ff @(posedge clk) begin
        if (fail) begin
            DEBUG_WARN("DUT", "Expected Failure Detected by DUT!");
        end
    end

    // FSM Transitions Monitor
    ble_setup_state_t prev_state;
    logic illegal_transition;
    
    always_comb begin
        illegal_transition = 1'b0;
        if (prev_state != dut.state) begin
            if (prev_state == IDLE && 
                !(dut.state inside {GET_CMD_NUMBER})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == GET_CMD_NUMBER && 
                !(dut.state inside {IDLE, SEND_CMD})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == SEND_CMD && 
                !(dut.state inside {IDLE, WAIT_ACK})) begin
                    illegal_transition = 1'b1;
            end
            if (prev_state == WAIT_ACK && 
                !(dut.state inside {IDLE, GET_ACK, SEND_CMD})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == GET_ACK &&
                !(dut.state inside {EVALUATE_ACK, WAIT_ACK})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == EVALUATE_ACK &&
                !(dut.state inside {IDLE, FLUSH_RX})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == FLUSH_RX &&
                !(dut.state inside {SEND_CMD})) begin    
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

    // Acknowledgement Monitor
    always @(posedge dut.ok_found or posedge dut.error_found) begin
        if (dut.ok_found) begin
            DEBUG_INFO("DUT", "Received: OK");
        end else if (dut.error_found) begin
            DEBUG_WARN("DUT", "Received: ERROR");
        end
    end

    // Setup Done Monitor
    always @(posedge setup_done) begin
        if (fail) begin 
            DEBUG_ERR("DUT", "Setup Done and Fail!", test_fail);
        end else begin
            DEBUG_INFO("DUT", "Setup Done!");
        end
    end

    // Stimulus
    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        if_top_link.load_regs = 1'b0;
        #(CLK_PERIOD_NS);
        reset_n(rst_n, CLK_PERIOD_NS);
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", $sformatf("Number of commands: %d", dut.cmd_number));
        
        DEBUG_INFO("TB", $sformatf("Simulating Setting up..."));
        setting_up = 1'b1;
        #(CLK_PERIOD_NS);
        setting_up = 1'b0;
        #(2*CLK_PERIOD_NS);
        test_zero_cmd = 1'b1;
        if_top_link.load_regs = 1'b1;
        DEBUG_INFO("DUT", $sformatf("Number of commands: %d", dut.cmd_number));
        #(CLK_PERIOD_NS);
        if_top_link.load_regs = 1'b0;
        setting_up = 1'b1;
        #(CLK_PERIOD_NS);
        setting_up = 1'b0;
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("TB", "Simulating Setting up... Done.");
        
        @(dut.state == WAIT_ACK);
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("TB", "Simulating Sending Commands...");
        DEBUG_INFO("DUT", "Sending: \"AT\\r\\n\"");
        #(10*BIT_PERIOD); //  A
        #(10*BIT_PERIOD); //  T
        #(10*BIT_PERIOD); //  \r
        #(10*BIT_PERIOD); //  \n
        DEBUG_INFO("TB", "Simulating Sending Commands... Done.");
        
        DEBUG_INFO("TB", "Testing Acknowledge Timeout...");
        #(CLK_PERIOD_NS);
        @(posedge if_tmr_inst.done);
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("TB", "Testing Acknowledge Timeout... Done.");
        
        DEBUG_INFO("TB", "Testing Retry to Send Commands...");
        DEBUG_INFO("DUT", "Sending: \"AT\\r\\n\"");
        #(10*BIT_PERIOD); //  A
        #(10*BIT_PERIOD); //  T
        #(10*BIT_PERIOD); //  \r
        #(10*BIT_PERIOD); //  \n
        DEBUG_INFO("TB", "Testing Retry to Send Commands... Done.");
        
        DEBUG_INFO("TB", "Testing Receiving Acknowledgement Ok...");
        DEBUG_INFO("TB", "Sending acknowledgement: \"OK\\r\\n\"");
        send_uart_string(ble_rx_pin, $sformatf("OK%c\n", 8'h0D), BIT_PERIOD);
        #(20*BIT_PERIOD); //  Timeout at FLUSH_RX
        DEBUG_INFO("TB", "Testing Receiving Acknowledgement Ok... Done.");
        
        DEBUG_INFO("TB", "Testing Sending Last Command...");
        DEBUG_INFO("DUT", "Sending: \"AT+NAME\\r\\n\"");
        #(10*BIT_PERIOD); //  A
        #(10*BIT_PERIOD); //  T
        #(10*BIT_PERIOD); //  +
        #(10*BIT_PERIOD); //  N
        #(10*BIT_PERIOD); //  A
        #(10*BIT_PERIOD); //  M
        #(10*BIT_PERIOD); //  E
        #(10*BIT_PERIOD); //  \r
        #(10*BIT_PERIOD); //  \n
        DEBUG_INFO("TB", "Sending acknowledgement: \"OK\\r\\n\"");
        send_uart_string(ble_rx_pin, $sformatf("OK%c\n", 8'h0D), BIT_PERIOD);
        #(20*BIT_PERIOD); //  Timeout at FLUSH_RX
        DEBUG_INFO("TB", "Testing Sending Last Command... Done.");
        
        DEBUG_INFO("TB", "Testing Receiving Acknowledgement Error...");    
        setting_up = 1'b1;
        #(2*CLK_PERIOD_NS);
        setting_up = 1'b0;
        @(dut.state == WAIT_ACK);
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Sending: \"AT\\r\\n\"");
        #(10*BIT_PERIOD); //  A
        #(10*BIT_PERIOD); //  T
        #(10*BIT_PERIOD); //  \r
        #(10*BIT_PERIOD); //  \n
        send_uart_string(ble_rx_pin, $sformatf("ERROR%c\n", 8'h0D), BIT_PERIOD);
        #(20*BIT_PERIOD); //  Timeout at FLUSH_RX
        DEBUG_INFO("TB", "Testing Receiving Acknowledgement Error... Done.");
        
        
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
        $dumpvars(0, ble_setup_tb); // Dump all signals in the testbench
    end

endmodule
