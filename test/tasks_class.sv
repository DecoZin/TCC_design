// ---------------------------------------------------------
// Module: Tasks for testbenches
// Description: All tasks used in the testbenches
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 1ps

`ifndef TASKS_SV
`define TASKS_SV
// DEBUG DISPLAY
package tb_tasks_pkg;
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

    class tb_tasks#(
        parameter DATA_DEPTH = 512,
        parameter DATA_WIDTH = 8
        );

        // Virtual interface handle for regs_if.master (can be assigned after object creation)
        virtual regs_if#(DATA_DEPTH, DATA_WIDTH) vif;

        // Constructor (optional: pass virtual interface handle on creation)
        function new(virtual regs_if#(DATA_DEPTH, DATA_WIDTH) vif_in = null);
            vif = vif_in;
        endfunction

        // Set the virtual interface handle if not passed in constructor
        function void set_vif(virtual regs_if#(DATA_DEPTH, DATA_WIDTH) vif_in);
            vif = vif_in;
        endfunction

        // Tasks as class methods

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
                DEBUG_INFO("UART TASK", $sformatf("Sending UART byte: %h (%c)", data, data));
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
                        DEBUG_ERR("UART TASK", "Stop bit not high");
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

        // Task that uses virtual interface member
        task automatic read_mem(
            output logic [7:0] data,
            input  int addr_val,
            input  int id
        );
            int timeout = 1000;
            begin
                if (vif == null) begin
                    DEBUG_ERR("MEM TASK", "Virtual interface not set in tb_tasks class!");
                    $finish;
                end
                
                // Drive read request
                // DEBUG_INFO("MEM TASK", $sformatf("Sending read req to addr = %0d", addr_val));

                vif.addr     = addr_val;
                vif.read_en  = 1;
                vif.write_en = 0;
                
                @(posedge vif.clk);
                #1;
                
                // Wait for data_ready
                while (vif.data_ready == 0 && timeout > 0) begin
                    @(posedge vif.clk);
                    // DEBUG_INFO("MEM TASK", $sformatf("Waiting for data_ready at addr %0d, timeout = %0d", addr_val, timeout));
                    timeout--;
                end
                
                if (timeout == 0) begin
                    DEBUG_ERR("MEM TASK", $sformatf("Timeout waiting for data_ready at address %0d!", addr_val));
                    $finish;
                end
                
                data = vif.read_data;
                
                // DEBUG_INFO("MEM TASK", $sformatf("[%t] Read: addr = %0d, data = 0x%02x (cmd ID = %0d)", addr_val, data, id));
                
                // Deassert
                vif.read_en = 0;
                @(posedge vif.clk);
            end
        endtask

        // NOT TESTED!!
        task automatic write_mem(
            input logic [7:0] data,
            input int addr_val,
            input int id
        );
            begin
                if (vif == null) begin
                    DEBUG_ERR("MEM TASK", "Virtual interface not set in tb_tasks class!");
                    $finish;
                end
        
                // Drive write request
                // DEBUG_INFO("MEM TASK", $sformatf("Writing 0x%02x to addr = %0d (cmd ID = %0d)", data, addr_val, id));
        
                vif.addr       = addr_val;
                vif.write_data = data;
                vif.write_en   = 1;
                vif.read_en    = 0;
        
                @(posedge vif.clk);
                #1;
        
                // Deassert write
                vif.write_en = 0;
                @(posedge vif.clk);
            end
        endtask
        

    endclass
endpackage

`endif // TASKS_SV
