// ---------------------------------------------------------
// Module: UART Module testbench
// Description: Testbench for the UART module
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

module uart_tb;
    logic clk   = 0;
    logic baud  = 0;
    logic rst_n = 1;
    logic i_rx_serial_ext;
    logic i_valid_tx;
    logic o_tx_serial_ext;
    logic o_valid_rx;
    logic o_done;
    logic o_rx_ready;
    logic o_tx_full;
    logic [2:0] uart_rx_state;
    logic [2:0] uart_tx_state;
    logic [1:0] rx_fifo_fe;
    logic [1:0] tx_fifo_fe;
    logic sel;
    logic [7:0] o_rx_sys_data;
    logic [7:0] byte_stream [];

    localparam real BAUD_RATE = 9600; // Baud rate
    localparam real CLK_FREQ = 50_000_000; // Clock frequency in Hz
    localparam int BIT_PERIOD = $rtoi(1e9 / BAUD_RATE); // = 104166.66 ns (when timescale is 1ns)
    localparam int CLK_PERIOD = $rtoi(1e9 / CLK_FREQ); // in ns

    // Instantiate the UART module
    UART #(.BAUD(BAUD_RATE), .CLK_F(CLK_FREQ)) uut (
        .clk(clk),
        .rst_n(rst_n),
        
        .i_test_mux(sel),                   // 0: normal | 1: test
        .i_transmit(1'b1),                  // Transmit data from RX to TX
        .i_rx_serial_ext(i_rx_serial_ext),
        .i_rx_fifo_rd_en(1'b1),
        .i_valid_tx(i_valid_tx),
        .i_tx_sys_data(8'h33),
        
        .o_tx_serial_ext(o_tx_serial_ext),
        .o_rx_sys_data(o_rx_sys_data),
        .o_valid_rx(o_valid_rx),
        .o_done(o_done),
        .o_rx_ready(o_rx_ready),
        .o_tx_full(o_tx_full),

        .uart_rx_state(uart_rx_state),
        .uart_tx_state(uart_tx_state),
        .rx_fifo_fe(rx_fifo_fe),
        .tx_fifo_fe(tx_fifo_fe)
    );



    logic timer0_en = 0;
    logic timer0_mode = 0;
    logic timer0_clear = 0;
    logic timer0_done;
    // timer #(.CLOCK_F(CLK_FREQ)) timer0 (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .i_enable(timer0_en),   // Enable to start counting
    //     .i_mode(timer0_mode),   // 0: One-shot, 1: Auto-reload
    //     .i_time(20'd1000),     // Time to count in µs up to 10s
    //     .i_clear(timer0_clear),             // Clear the counter
    //     .o_done(timer0_done)               // Signal after counting
    // );

    logic timer1_en = 0;
    logic timer1_mode = 0;
    logic timer1_clear = 0;
    logic timer1_done;
    // timer #(.CLOCK_F(CLK_FREQ)) timer1 (
    //     .clk(clk),
    //     .rst_n(rst_n),
    //     .i_enable(timer1_en),   // Enable to start counting
    //     .i_mode(timer1_mode),   // 0: One-shot, 1: Auto-reload
    //     .i_time(20'd1000),     // Time to count in µs up to 10s
    //     .i_clear(timer1_clear),             // Clear the counter
    //     .o_done(timer1_done)               // Signal after counting
    // );

    // Generate the clock signals
    initial fork
        generate_clock(clk, {{32{$rtoi(CLK_PERIOD / 2)[31]}},$rtoi(CLK_PERIOD / 2)});
    join_none

    initial fork
        generate_clock(baud, {{32{BIT_PERIOD[31]}},BIT_PERIOD});
    join_none

    // Continuous monitoring of the TX pin
    int i;
    byte received;

    always @(negedge o_tx_serial_ext) begin
        DEBUG_INFO("UART", "Falling edge detected (start bit)");
        #(BIT_PERIOD + 10);

        for (i = 0; i < 8; i++) begin
            received[i] <= o_tx_serial_ext;
            #(BIT_PERIOD);
        end

        if (o_tx_serial_ext !== 1) begin
            DEBUG_ERR("UART", "UART ERROR: Stop bit not high");
        end else begin
            DEBUG_INFO("UART", $sformatf("UART Received: %h", received));
        end

        #(BIT_PERIOD); // Wait stop bit
    end

    initial begin
        DEBUG_INFO("TB", "Testbench started!.");
        sel = 1; // Loopback mode
        i_rx_serial_ext = 1;
        i_valid_tx = 0;
        rst_n = 1;
        #20;

        reset_n(rst_n);
        #200;

        // Send a byte to UART module
        send_uart_byte(i_rx_serial_ext, 8'h55, {{32{BIT_PERIOD[31]}},BIT_PERIOD}); // Send 0x55
        #(10 * BIT_PERIOD); // Wait for transmission to complete

        // TX Response
        #(10*BIT_PERIOD);
        sel = 0; // Normal mode
        DEBUG_INFO("TB", "Sending internal UART byte: 33");
        i_valid_tx = 1;
        #(CLK_PERIOD);
        #(CLK_PERIOD);
        i_valid_tx = 0;

        // TX Response again
        #(10*BIT_PERIOD);

        // Loopback mode again
        #20;
        sel = 1;
        // Example byte stream
        byte_stream = '{8'h00, 8'h01, 8'h02, 8'h03, 8'h04, 8'h05, 8'h06, 8'h07, 8'h08, 8'h09, 8'h0A}; 
        send_uart_stream(i_rx_serial_ext, byte_stream, 11, {{32{BIT_PERIOD[31]}},BIT_PERIOD}); 
        #(10*BIT_PERIOD);

        // End simulation
        #20;
        $finish;
    end

    initial begin
        $dumpfile("waveform.vcd"); // Specify the VCD file name
        $dumpvars(0, uart_tb);     // Dump all signals in the testbench
    end
endmodule
