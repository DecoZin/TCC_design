// ---------------------------------------------------------
// Module: Tasks for testbenches
// Description: All tasks used in the testbenches
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef TASKS_SV
`define TASKS_SV
// DEBUG DISPLAY
package tasks_pkg;

    task automatic DEBUG_PASS();
        $display("\033[1;32mTEST PASSED!\033[0m");
    endtask

    task automatic DEBUG_FAIL();
        $display("\033[1;31mTEST FAILED!\033[0m");
    endtask

    task automatic DEBUG_INFO(input string tag, msg);
        $display("\033[1;32m[INFO][%4s]\033[0m[%9t ns] %s", tag, ($time/10), msg);
    endtask

    task automatic DEBUG_WARN(input string tag, msg);
        $display("\033[1;33m[WARN][%4s]\033[0m[%9t ns] %s", tag, ($time/10), msg);
    endtask

    task automatic DEBUG_ERR(
        input string tag, msg,
        ref logic fail_flag
        );
        $display("\033[1;31m[ERRO][%4s][%9t ns] %s\033[0m", tag, ($time/10), msg);
        set_failure(fail_flag);
    endtask

    // DEBUG WRITE
    task automatic DEBUG_INFOWT(input string tag, msg);
        $write("\033[1;32m[INFO][%4s]\033[0m[%9t ns] %s", tag, ($time/10), msg);
    endtask

    task automatic DEBUG_WARNWT(input string tag, msg);
        $write("\033[1;33m[WARN][%4s]\033[0m[%9t ns] %s", tag, ($time/10), msg);
    endtask

    task automatic DEBUG_ERRWT(
        input string tag, msg,
        ref logic fail_flag
        );
        $write("\033[1;31m[ERRO][%4s][%9t ns] %s\033[0m", tag, ($time/10), msg);
        set_failure(fail_flag);
    endtask

    // DEBUG WRITE WITH NO HEADER
    task automatic DEBUG_INFOW(input string msg);
        $write("%s", msg);
    endtask

    task automatic DEBUG_WARNW(input string msg);
        $write("%s", msg);
    endtask

    task automatic DEBUG_ERRW(
        input string msg,
        ref logic fail_flag
        );
        $write("\033[1;31m %s \033[0m", msg);
        set_failure(fail_flag);
    endtask

    // Tasks
    task automatic displayTask();
        DEBUG_INFO("TASK", "displayTask() called.");
    endtask

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
        input  int duration = 20
    );
        DEBUG_INFO("TASK", "Resetting system.");
        rst_n = 0;
        #(duration);
        rst_n = 1;
    endtask

    task automatic send_uart_byte(
        ref logic  rx_serial,
        input  logic [7:0] data,
        input  int  bit_period = 104167 // 9600 baud rate for 1ns timescale
    );
        integer i;
        begin
            if (data == 8'h0D) begin
                DEBUG_INFO("UART TASK", "Sending UART byte: 0x0D (\\r)");
            end else if (data == 8'h0A) begin
                DEBUG_INFO("UART TASK", "Sending UART byte: 0x0A (\\n)");
            end else begin
                if ((data >= "a" && data <= "z") ||
                    (data >= "A" && data <= "Z") || 
                    (data >= "0" && data <= "9") ||
                    (data == ",") || (data == "!") ||
                    (data == "+") || (data == ":")) begin
                    DEBUG_INFO("UART TASK", $sformatf("Sending UART byte: 0x%h (%c)", data, data));
                end else begin
                    DEBUG_INFO("UART TASK", $sformatf("Sending UART byte: 0x%h", data));
                end
            end
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
            input int bit_period,
            input bit clk,
            ref logic fail_flag
        );
        logic prev_tx = 1;
        byte received;
        integer i;
        begin
            forever begin
                // Wait for falling edge of TX (start bit)
                @(negedge tx);
                DEBUG_INFO("UART TASK", "Falling edge detected (start bit)");

                // Wait for 1.5 bit periods to get past the start bit
                #(bit_period * 3/2);
                DEBUG_INFO("UART TASK", "After 1.5 bit periods");

                // Sample 8 data bits
                for (i = 0; i < 8; i++) begin
                    received[i] = tx; // Capture each data bit
                    #(bit_period); // Wait for next bit
                end

                // Check the stop bit
                if (tx !== 1) begin
                    DEBUG_ERR("UART TASK", "Stop bit not high", fail_flag);
                end else begin
                    DEBUG_INFO("UART TASK", $sformatf("UART Received: %c (0x%02h)", received, received));
                end

                #(bit_period); // Finish stop bit
            end
        end
    endtask

    task automatic send_uart_stream(
        ref    logic rx_serial,
        input  logic [7:0] data_array[],
        input  int num_bytes,
        input  int bit_period = 104167
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
        input int bit_period = 104167
    );
        int i;
        begin
            for (i = 0; i < str.len(); i++) begin
                send_uart_byte(rx_serial, str[i], bit_period);
                #(bit_period/2);
            end
        end
    endtask       
    
    task automatic set_failure(
        ref logic fail_flag
    );
        fail_flag = 1;
    endtask

endpackage

`endif // TASKS_SV
