// ---------------------------------------------------------
// Module: Processor Module testbench
// Description: Testbench for the Processor
// Author: André Lamego
// Date: 2025-10-22
// ---------------------------------------------------------
`timescale 1ns / 100ps

module processor_tb;
    import tasks_pkg::*;	
    import processor_types_pkg::*;
    import ei_mem_pkg::*;

    // Local Parameters
    localparam int CLK_FREQ_HZ   = 50_000_000;
    localparam int CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ_HZ;
    localparam int DATA_WIDTH = 8;
    localparam int SR_DATA_DEPTH = 43;
    localparam int BUS_DATA_DEPTH = 1024;
    localparam real BAUD_RATE = 9600; // Baud rate
    localparam int BIT_PERIOD = $rtoi(1e9 / BAUD_RATE); // = 104_166.66 ns (when timescale is 1ns)
    localparam int BYTE_PERIOD = 10*BIT_PERIOD;
    localparam int TIMEOUT_US = $rtoi(15*BIT_PERIOD/1000);

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
        .DATA_DEPTH(BUS_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_bus(.clk(clk), .rst_n(rst_n));

    regs_int_if #(
        .DATA_DEPTH(SR_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link(.clk(clk), .rst_n(rst_n));

    regs_int_if #(
        .DATA_DEPTH(BUS_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_int_bus(.clk(clk), .rst_n(rst_n));


    tmr_if if_processor_tmr(.clk(clk), .rst_n(rst_n));;

    // Connection Monitor module Instantiation
    logic connect = 1'b0;
    logic disconnect = 1'b0;
    logic setup_done = 1'b0;
    logic [7:0] image_data;
    logic img_data_valid;
    
    processor dut (
        // clock / reset
        .clk(clk),
        .rst_n(rst_n),
        // Interfaces regs (master modport)
        .if_special_regs(if_spec_regs.master), 
        .if_bus(if_bus.master),
        // timer interface (controller modport)
        .if_timer(if_processor_tmr.controller),
        // connection signals (from connection monitor)
        .connect(connect),
        .disconnect(disconnect),
        // UART RX (from external interface)
        .data_valid(valid_rx),
        .data_ready(rx_fifo_ready),
        .data_byte(rx_sys_data),
        .get_data(rx_fifo_rd_en),
        // UART TX (to external interface)
        .tx_full(tx_full),
        .ack_byte(tx_sys_data),
        .ack_valid(tx_valid),
        // Image path (to cornea_opaca)
        .image_data(image_data),
        .img_data_valid(img_data_valid),
        // timing config (from regs)
        .regs_opds_time_count(regs_opds_time_count),
        .regs_delay_time_count(regs_delay_time_count)
    );

    // Timer Instantiation
    timer #(.CLOCK_F(CLK_FREQ_HZ)) processor_timer (
        .if_t(if_processor_tmr.timer),
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

    // "Bus" Instantiation (Just a Register File with some default values)
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(BUS_DATA_DEPTH)
    ) bus (
        .if_regs_inst(if_bus.slave),
        .if_top_link(if_int_bus.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    logic [23:0] regs_opds_time_count;
    logic [23:0] regs_delay_time_count;
    
    always_comb begin
        if_top_link.mode_mask = '0; // All special registers are Read-Write
        if_int_bus.mode_mask = '0;  // All bus addresses are Read-Write
        if_top_link.load_regs = '0; // We won't load any special registers
        if_int_bus.load_regs = '0;  // We won't load any bus registers
        
        if_int_bus.regi = '{default: '0};  // Default values for all bus addresses

        if_int_bus.regi[{B_SENS_OFFSET,A_NMEAS}] = 8'h04;
        if_int_bus.regi[{B_SENS_OFFSET,A_GETMEAS}] = 8'hFA;
        if_int_bus.regi[{B_SENS_OFFSET,A_ALLMEAS}] = 8'h00;
        if_int_bus.regi[{B_SENS_OFFSET,(A_ALLMEAS+8'h01)}] = 8'h01;
        if_int_bus.regi[{B_SENS_OFFSET,(A_ALLMEAS+8'h02)}] = 8'h02;


        if_top_link.regi = '{default: '0}; // Default values for all special registers
        
        if_top_link.regi[EIR_OPDS_TMR0] =  8'(TIMEOUT_US);
        if_top_link.regi[EIR_OPDS_TMR1] =  8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_OPDS_TMR2] =  8'(TIMEOUT_US >> 16);
        if_top_link.regi[EIR_DELAY_TMR0] = 8'(TIMEOUT_US);
        if_top_link.regi[EIR_DELAY_TMR1] = 8'(TIMEOUT_US >> 8);
        if_top_link.regi[EIR_DELAY_TMR2] = 8'(TIMEOUT_US >> 16);

        regs_opds_time_count  = {if_top_link.rego[EIR_OPDS_TMR2],
                                 if_top_link.rego[EIR_OPDS_TMR1],
                                 if_top_link.rego[EIR_OPDS_TMR0]};
        regs_delay_time_count = {if_top_link.rego[EIR_DELAY_TMR2],
                                 if_top_link.rego[EIR_DELAY_TMR1],
                                 if_top_link.rego[EIR_DELAY_TMR0]};
    end

    // UART Instantiation
    logic baud = 1'b0; // Signal toggling at baud rate
    logic ble_rx_pin = 1'b1;
    wire [7:0] rx_sys_data;
    wire tx_valid;
    wire [7:0] tx_sys_data;
    logic ble_tx_pin;
    wire valid_rx;
    wire rx_fifo_ready;
    wire rx_fifo_rd_en;
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
        .i_transmit(1'b0),  // Transmit data from RX to TX (not used)
        .i_rx_serial_ext(ble_rx_pin),
        .i_rx_fifo_rd_en(rx_fifo_rd_en),
        .i_valid_tx(tx_valid),
        .i_tx_sys_data(tx_sys_data),
        
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

    // Monitor for Processor Monitor Timer
    always_ff @(posedge clk) begin
        if (if_processor_tmr.done) begin
            if (dut.state == S_DELAY) begin
                DEBUG_WARN("DUT", "Delay Timeout Hit!");
            end else begin
                DEBUG_WARN("DUT", "Getting Operands Timeout Hit!");
            end
        end
    end

    // FSM Transitions Monitor
    processor_state_t prev_state;
    logic illegal_transition;
    
    always_comb begin
        illegal_transition = 1'b0;
        if (prev_state != dut.state) begin
            if (prev_state == S_DISCONNECTED && 
                !(dut.state inside {S_IDLE})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == S_IDLE && 
                !(dut.state inside {S_DISCONNECTED, S_DECODE_CMD})) begin
                illegal_transition = 1'b1;
            end
            if (prev_state == S_DECODE_CMD && 
                !(dut.state inside {S_IDLE, S_GET_OPDS, S_MEM})) begin
                    illegal_transition = 1'b1;
            end
            if (prev_state == S_GET_OPDS && 
                !(dut.state inside {S_IDLE, S_MEM})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == S_MEM &&
                !(dut.state inside {S_DELAY})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == S_DELAY &&
                !(dut.state inside {S_ACK})) begin    
                illegal_transition = 1'b1;
            end
            if (prev_state == S_ACK &&
                !(dut.state inside {S_IDLE})) begin    
                illegal_transition = 1'b1;
            end
        end    
    end

    always_ff @(posedge clk) begin
        prev_state <= dut.state;
        if (!rst_n) begin
            DEBUG_INFO("DUT", $sformatf("FSM: %s", dut.state.name())); 
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

    // Register Write Monitor
    always @(posedge clk) begin
        if (if_spec_regs.write_en) begin
            DEBUG_INFO("DUT", $sformatf("Special Registers Write: ADDR: 0x%x, DATA: 0x%x", if_spec_regs.addr, if_spec_regs.write_data));
        end
    end

    always @(posedge clk) begin
        if (if_bus.write_en) begin
            DEBUG_INFO("DUT", $sformatf("Bus Write: ADDR: 0x%x, DATA: 0x%x", if_bus.addr, if_bus.write_data));
        end
    end

    // Opcode Detection Monitor
    always @(posedge clk) begin
        if (prev_state == S_DECODE_CMD)
        if (dut.state == S_GET_OPDS || dut.state == S_MEM) begin
            DEBUG_INFO("DUT", $sformatf("Opcode Detected: %s", dut.cmd_opcode.name()));
        end else if (dut.state == S_IDLE) begin
            DEBUG_WARN("DUT", $sformatf("Opcode invalid: 0x%x", dut.cmd_opcode));
        end
    end

    // Image data Monitor
    always @(posedge clk) begin
        if (img_data_valid) begin
            DEBUG_INFO("DUT", $sformatf("Image Data: 0x%x", image_data));
        end
    end


    // Stimulus
    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        #(CLK_PERIOD_NS);
        reset_n(rst_n, CLK_PERIOD_NS);
        connect = 1'b1;
        #(CLK_PERIOD_NS); 
        connect = 1'b0;
        #(CLK_PERIOD_NS); 
        
        DEBUG_INFO("TB", "Testing Commands...");
        DEBUG_INFO("TB", "Sending INTERREGW, to write data 0xF1 in register EIR_TEST.");
        DEBUG_INFO("TB", "Sending: \"0x02 0x00 0xF1\"...");
        send_uart_stream(ble_rx_pin, {OPC_INTERREGW, EIR_TEST, 8'hF1}, 3, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending INTERREGR, to read data from register EIR_TEST.");
        DEBUG_INFO("TB", "Sending: \"0x11 0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_INTERREGR, EIR_TEST}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);
        
        DEBUG_INFO("TB", "Sending GLOBALREGW, to write data 0xF2 in register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x22 0x00 0xF2\"...");
        send_uart_stream(ble_rx_pin, {OPC_GLOBALREGW, 8'h00, 8'hF2}, 3, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending GLOBALREGR, to read data from register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x31 0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_GLOBALREGR, 8'h00}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending JPEGREGW, to write data 0xF3 in register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x42 0x00 0xF3\"...");
        send_uart_stream(ble_rx_pin, {OPC_JPEGREGW, 8'h00, 8'hF3}, 3, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending JPEGREGR, to read data from register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x51 0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_JPEGREGR, 8'h00}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending DISPLAYREGW, to write data 0xF4 in register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x62 0x00 0xF4\"...");
        send_uart_stream(ble_rx_pin, {OPC_DISPLAYREGW, 8'h00, 8'hF4}, 3, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending DISPLAYREGR, to read data from register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x71 0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_DISPLAYREGR, 8'h00}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending SENSREGW, to write data 0xF5 in register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x82 0x00 0xF5\"...");
        send_uart_stream(ble_rx_pin, {OPC_SENSREGW, 8'h00, 8'hF5}, 3, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending SENSREGR, to read data from register 0x00.");
        DEBUG_INFO("TB", "Sending: \"0x91 0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_SENSREGR, 8'h00}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending GETMEASNUM, to read data from register NMEAS.");
        DEBUG_INFO("TB", "Sending: \"0xA0\"...");
        send_uart_stream(ble_rx_pin, {OPC_GETMEASNUM}, 1, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending GETMEAS, to read data from register 0x04.");
        DEBUG_INFO("TB", "Sending: \"0xB0\"...");
        send_uart_stream(ble_rx_pin, {OPC_GETMEAS, 8'h04}, 2, BIT_PERIOD);
        #(11*BYTE_PERIOD);
        
        DEBUG_INFO("TB", "Sending GETALLMEAS, to read data from all registers with measurements in Sensor Block.");
        DEBUG_INFO("TB", "Sending: \"0xC0\"...");
        send_uart_stream(ble_rx_pin, {OPC_GETALLMEAS}, 1, BIT_PERIOD);
        #(20*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending RTMEAS, to read data from register A_GETMEAS.");
        DEBUG_INFO("TB", "Sending: \"0xD0\"...");
        send_uart_stream(ble_rx_pin, {OPC_RTMEAS}, 1, BIT_PERIOD);
        #(11*BYTE_PERIOD);

        DEBUG_INFO("TB", "Sending DISPLAYIMG, to send 10 image data to display.");
        DEBUG_INFO("TB", "Sending: \"0xEF 0x0A 0x00 0x01 ... 0x09\"...");
        send_uart_stream(ble_rx_pin, {OPC_DISPLAYIMG, 8'h0A, 8'h00, 8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07, 8'h08, 8'h09}, 12, BIT_PERIOD);
        #(10*BYTE_PERIOD);        

        DEBUG_INFO("TB", "Sending invalid opcode.");
        DEBUG_INFO("TB", "Sending: \"0x00\"...");
        send_uart_stream(ble_rx_pin, {OPC_NOP}, 1, BIT_PERIOD);
        #(BIT_PERIOD);
        
        DEBUG_INFO("TB", "Testing Commands... Done.");

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
