// ---------------------------------------------------------
// Module: Tasks for testbenches
// Description: All tasks used in the testbenches
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef TASKS_SV
`define TASKS_SV

task automatic DEBUG_PASS();
    $display("\033[1;32mTEST PASSED!\033[0m");
endtask
task automatic DEBUG_FAIL();
    $display("\033[1;32mTEST FAILED!\033[0m");
endtask
task automatic DEBUG_INFO(input string tag, msg);
    $display("\033[1;32m[INFO][%s]\033[0m[%09t] %s", tag, ($time/1000), msg);
endtask
task automatic DEBUG_WARN(input string tag, msg);
    $display("\033[1;33m[WARN][%s]\033[0m[%09t] %s", tag, ($time/1000), msg);
endtask
task automatic DEBUG_ERR(input string tag, msg);
    $display("\033[1;31m[ERROR][%s][%09t] %s\033[0m", tag, ($time/1000), msg);
endtask
// DEBUG WRITE
task automatic DEBUG_INFOWT(input string tag, msg);
    $write("\033[1;32m[INFO][%s]\033[0m[%09t] %s", tag, ($time/1000), msg);
endtask
task automatic DEBUG_WARNWT(input string tag, msg);
    $write("\033[1;33m[WARN][%s]\033[0m[%09t] %s", tag, ($time/1000), msg);
endtask
task automatic DEBUG_ERRWT(input string tag, msg);
    $write("\033[1;31m[ERROR][%s][%09t] %s\033[0m", tag, ($time/1000), msg);
endtask
// DEBUG WRITE WITH NO HEADER
task automatic DEBUG_INFOW(input string msg);
    $write("%s", msg);
endtask
task automatic DEBUG_WARNW(input string msg);
    $write("%s", msg);
endtask
task automatic DEBUG_ERRW(input string msg);
    $write("\033[1;31m %s \033[0m", msg);
endtask

task automatic displayTask();
    DEBUG_INFO("TASK", "displayTask() called.");
endtask //automatic

task automatic generate_clock(
    ref logic clk,
    input time half_period,
    input int cycles = 0  // Optional: 0 = run forever
);
    int count = 0;
    clk = 0;
    forever begin
        #(half_period);
        clk = ~clk;

        if (cycles > 0) begin
            count++;
            if (count >= cycles * 2) break;
        end
    end
endtask

task automatic reset_n(
    ref logic rst_n,
    input  time duration = 20
);
    rst_n = 0;
    #(duration);
    rst_n = 1;
endtask

task automatic send_uart_byte(
    ref logic  rx_serial,
    input  logic [7:0] data,
    input  time  bit_period = 104167 // 9600 baud rate for 1ns timescale
);
    integer i;
    begin
        DEBUG_INFO("UART", $sformatf("Sending byte: %h", data));
        rx_serial = 0; // Start bit
        #(bit_period); // Wait for 1 bit period

        for (i = 0; i < 8; i++) begin
            rx_serial = data[i]; // Data bits
            #(bit_period); // Wait for 1 bit period
        end

        rx_serial = 1; // Stop bit
        #(bit_period); // Wait for 1 bit period
    end  
endtask

task automatic monitor_uart_rx(
        input logic tx, 
        input time bit_period,
        input bit clk
    );
    logic prev_tx = 1;
    byte received;
    integer i;
    begin
        forever begin
            // Wait for falling edge of TX (start bit)
            @(negedge tx);
            $display("[%t] Falling edge detected (start bit)", $time);

            // Wait for 1.5 bit periods to get past the start bit
            #(bit_period * 3/2);
            $display("[%t] After 1.5 bit periods", $time);

            // Sample 8 data bits
            for (i = 0; i < 8; i++) begin
                received[i] = tx; // Capture each data bit
                #(bit_period); // Wait for next bit
            end

            // Check the stop bit
            if (tx !== 1) begin
                $display("[%t] UART ERROR: Stop bit not high", $time);
            end else begin
                $display("[%t] UART Received: %c (0x%02h)", $time, received, received);
            end

            #(bit_period); // Finish stop bit
        end
    end
endtask


task automatic send_uart_stream(
    ref    logic rx_serial,
    input  logic [7:0] data_array[],
    input  int num_bytes,
    input  time bit_period = 104167
);
    int i;
    begin
        for (i = 0; i < num_bytes; i++) begin
            send_uart_byte(rx_serial, data_array[i], bit_period);
            #(bit_period/2); // small pause between bytes
        end
    end
endtask

task automatic send_uart_string(
    ref logic rx_serial,
    input string str,
    input time bit_period = 104167
);
    int i;
    begin
        for (i = 0; i < str.len(); i++) begin
            send_uart_byte(rx_serial, str[i], bit_period);
            #(bit_period/2);
        end
    end
endtask

`endif // TASKS_SV
