// ---------------------------------------------------------
// Module: BLE Setup Controller Module testbench
// Description: Testbench for the BLE Setup Controller
// Author: André Lamego
// Date: 2025-10-15
// ---------------------------------------------------------
`timescale 1ns / 100ps

module ble_setup_controller_tb;
    import tasks_pkg::*;	
    import ble_ctrl_types_pkg::*;

    // Local Parameters
    localparam logic [23:0] SLP_TIMER_US = 24'd1;
    localparam int CLK_FREQ_HZ   = 50_000_000;
    localparam int CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ_HZ;
    localparam int CLKS_AT_DONE  = (SLP_TIMER_US * 1_000) / CLK_PERIOD_NS;

    logic test_fail = 1'b0;

    // Clock and Reset
    logic clk;
    logic rst_n;

    // Instantiate Deep Sleep Timer Interface
    tmr_if if_ds_timer(.clk(clk), .rst_n(rst_n));

    // Instantiate the BLE Setup Controller module
    logic programming = 1'b0;
    logic direct_conn = 1'b0;
    logic setup_done = 1'b0;
    logic fail = 1'b0;
    logic time_out = 1'b0;
    logic connect = 1'b0;
    logic disconnect = 1'b0;
    logic error_pulse = 1'b0;
    logic [1:0] error_code = 2'b00;
    logic en_cmd_mem_wr;
    logic setting_up;
    logic [1:0] mux_rx_setup;
    logic mux_tx_setup;
    logic mux_transceiver;
    logic tx_full = 1'b0;
    logic tx_valid;
    logic [7:0] tx_data;

    ble_setup_controller dut (
        // Interfaces
        .if_ds_timer(if_ds_timer.controller),
        .regs_slp_time_count(SLP_TIMER_US),       
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
        .time_out(time_out),
        // External Signals
        .programming(programming),
        .direct_conn(direct_conn),
        // Command Memory Interface
        .en_cmd_mem_wr(en_cmd_mem_wr),
        .error_pulse(error_pulse),
        .error_code(error_code),
        // UART TX Interface
        .tx_valid(tx_valid),
        .tx_data(tx_data),
        .tx_full(tx_full),
        // Muxes
        .mux_rx_setup(mux_rx_setup),
        .mux_tx_setup(mux_tx_setup),
        .mux_transceiver(mux_transceiver)
    );

    // Instantiate Deep Sleep Timer Module
    timer #(.CLOCK_F(CLK_FREQ_HZ)) timer_deep_sleep (
        .if_t(if_ds_timer.timer),
        .clk(clk),
        .rst_n(rst_n)
    );

    // Generate the clocks
    initial fork
        generate_clock(clk, {{32{$rtoi(CLK_PERIOD_NS / 2)[31]}},$rtoi(CLK_PERIOD_NS / 2)});
    join_none

    // Global timeout for stuck simulation
    initial begin
        #10_000_000;
        DEBUG_ERR("TB", "Global timeout hit! Simulation stuck.", test_fail);
        $finish;
    end

    // Timer done monitor
    int tmr_clk_cnt = 0;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tmr_clk_cnt <= 0;
        end else begin
            if (if_ds_timer.clear) begin
                tmr_clk_cnt <= 0;
            end else if (if_ds_timer.enable) begin
                tmr_clk_cnt <= tmr_clk_cnt + 1;
                if (tmr_clk_cnt-1 == CLKS_AT_DONE) begin
                    if (if_ds_timer.done) begin
                        DEBUG_INFO("TB", $sformatf("Wake up after %0d ms.", tmr_clk_cnt * CLK_PERIOD_NS / 1000));
                        tmr_clk_cnt <= 0;
                    end else begin
                        DEBUG_ERR("TB", $sformatf("Did not wake up after %0d ms.", tmr_clk_cnt * CLK_PERIOD_NS / 1000), test_fail);
                    end
                end
            end 
        end
    end

    
    // FSM Transitions Monitor
    ble_ctrl_state_t prev_state;
    logic illegal_transition;
    
    always_comb begin
        illegal_transition = 1'b0;
        if (prev_state != dut.state) begin
            if (prev_state == IDLE && 
                !(dut.state inside {PROGRAMMING, SETUP, CONNECTED})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == PROGRAMMING && 
                !(dut.state inside {IDLE, ERROR_ACK})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == ERROR_ACK && 
                !(dut.state inside {PROGRAMMING})) begin
                    illegal_transition = 1'b1;
            end
            if (prev_state == SETUP && 
                !(dut.state inside {IDLE, ADVERTISEMENT})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == ADVERTISEMENT &&
                !(dut.state inside {CONNECTED, IDLE})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == CONNECTED &&
                !(dut.state inside {IDLE})) begin    
                illegal_transition = 1'b1;
            end
        end    
    end

    always_ff @(posedge clk or negedge rst_n) begin
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

    // Signals Monitor
    logic incorrect_signals;

    always_comb begin
        incorrect_signals = 1'b0;
        case (dut.state)
            IDLE: begin
                if (en_cmd_mem_wr ||
                    setting_up ||
                    mux_transceiver ||
                    tx_valid) begin
                        incorrect_signals = 1'b1;
                end
            end
            PROGRAMMING: begin
                if (!en_cmd_mem_wr ||
                    setting_up ||
                    mux_transceiver ||
                    mux_rx_setup != 2'b01 ||
                    mux_tx_setup != 1'b1 ||
                    tx_valid) begin
                        incorrect_signals = 1'b1;
                end
                end
            ERROR_ACK: begin
                if (en_cmd_mem_wr ||
                    setting_up ||
                    mux_transceiver ||
                    mux_rx_setup != 2'b01 ||
                    mux_tx_setup != 1'b1) begin
                        incorrect_signals = 1'b1;
                end
            end
            SETUP: begin
                if (en_cmd_mem_wr ||
                    !setting_up ||
                    mux_transceiver ||
                    mux_rx_setup != 2'b00 ||
                    mux_tx_setup != 1'b0 ||
                    tx_valid) begin
                        incorrect_signals = 1'b1;
                end
            end
            ADVERTISEMENT: begin
                if (en_cmd_mem_wr ||
                    setting_up ||
                    mux_transceiver ||
                    mux_rx_setup != 2'b10 ||
                    tx_valid) begin
                        incorrect_signals = 1'b1;
                end
            end
            CONNECTED: begin
                if (en_cmd_mem_wr ||
                    setting_up ||
                    !mux_transceiver) begin
                        incorrect_signals = 1'b1;
                end
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (incorrect_signals) begin
            DEBUG_ERR("DUT", $sformatf("Incorrect signals in state %s", dut.state.name()), test_fail);
        end
    end

    // TX error message monitor
    always_ff @(posedge clk) begin
        if (tx_valid) begin
            if (tx_data == {($bits(tx_data)-$bits(error_code ))'('0),error_code}) begin
            DEBUG_INFO("DUT", $sformatf("Received error code: %0d", tx_data));
            end else begin
            DEBUG_ERR("DUT", $sformatf("Received incorrect error code: %0d", tx_data), test_fail);
            end
        end
    end

    // Stimulus
    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        #(CLK_PERIOD_NS);
        reset_n(rst_n);
        #(CLK_PERIOD_NS);
        programming = 1'b1; 
        direct_conn = 1'b0;

        // Testing Deep Sleep Timer
        DEBUG_INFO("TMR", "Testing Deep Sleep Timer, set to 1ms...");
        @(posedge if_ds_timer.done);
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("TMR", "Testing Deep Sleep Timer, set to 1ms... Done.");
        #(5*CLK_PERIOD_NS);
        
        // Testing BLE Setup Controller
        DEBUG_INFO("DUT", "Testing programming signals...");
        DEBUG_INFO("DUT", "Testing error code '1'.");
        #(3*CLK_PERIOD_NS);
        error_pulse = 1'b1;
        error_code = 2'b01;
        #(CLK_PERIOD_NS);
        error_pulse = 1'b0;
        #(3*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing error code '2'.");
        error_pulse = 1'b1;
        error_code = 2'b10;
        #(CLK_PERIOD_NS);
        error_pulse = 1'b0;
        #(3*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing error code '3'.");
        error_pulse = 1'b1;
        error_code = 2'b11;
        #(CLK_PERIOD_NS);
        error_pulse = 1'b0;
        #(3*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing programming signals... Done.");
        programming = 1'b0;

        @(posedge if_ds_timer.done);
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing setup signals...");
        #(5*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing setup signals... Done.");

        DEBUG_INFO("DUT", "Simulating failure setup...");
        fail = 1'b1;
        #(CLK_PERIOD_NS);
        fail = 1'b0;
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Simulating failure setup... Done.");

        @(posedge if_ds_timer.done);
        #(2*CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Simulating successful setup...");
        setup_done = 1'b1;
        #(CLK_PERIOD_NS);
        setup_done = 1'b0;
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Simulating successful setup... Done.");

        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing advertisement signals...");
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing advertisement signals... Done.");
        
        connect = 1'b1;
        DEBUG_INFO("DUT", "Testing connected signals...");
        #(CLK_PERIOD_NS);
        connect = 1'b0;
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing connected signals... Done.");
        
        #(CLK_PERIOD_NS);
        disconnect = 1'b1;
        #(CLK_PERIOD_NS);
        disconnect = 1'b0;
        DEBUG_INFO("DUT", "Testing direct connection...");
        direct_conn = 1'b1;
        #(CLK_PERIOD_NS);
        direct_conn = 1'b0;
        #(CLK_PERIOD_NS);
        DEBUG_INFO("DUT", "Testing direct connection... Done.");
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

        // Dump waveform
    initial begin
        $dumpfile("waveform.fst");
        $dumpvars(0, ble_setup_controller_tb);
    end

endmodule
