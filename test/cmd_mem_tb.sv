// ---------------------------------------------------------
// Module: Command Memory Testbench
// Description: Testbench for the Command Memory module
// Author: André Lamego
// Date: 2025-05-08
// ---------------------------------------------------------
`timescale 1ns / 100ps

module cmd_mem_tb;
    import tasks_pkg::*;	
    import cmd_mem_pkg::*; 

    // Local Parameters
    localparam int CLK_FREQ_HZ = 50_000_000; // Clock frequency in Hz
    localparam int CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ_HZ; // in ns
    localparam int BAUD_RATE = 9600; // Baud rate
    localparam int BIT_PERIOD = $rtoi(1e9 / BAUD_RATE); // = 104_166.66 ns (when timescale is 1ns)
    localparam int CMD_WIDTH = 32;
    localparam int CMD_DEPTH = 16;
    localparam int DATA_WIDTH = 8;
    localparam int CMD_DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;
    
    // SIMULATION TIMEOUT
    localparam int SIMULATION_TIMEOUT = 10000*BIT_PERIOD;
    
    // Clock and Reset
    logic clk   = 0;
    logic rst_n = 1;

    // Error flag
    logic test_fail = 1'b0;

    // Interfaces instantiation        
    regs_if #(
        .DATA_DEPTH(CMD_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_regs_inst(.clk(clk), .rst_n(rst_n));

    // DUT Instatiation
    logic programming;
    logic rx_ready;
    logic rx_fifo_rd_en;
    logic [7:0] rx_sys_data;
    logic [1:0] error_code;
    logic error_pulse;
    
    cmd_memory #(
        .CMD_WIDTH(CMD_WIDTH),   // Number of bytes per command
        .CMD_DEPTH(CMD_DEPTH)    // Number of commands in memory
    ) dut (
        .if_regs_inst(if_regs_inst.slave),
        
        .clk(clk),
        .rst_n(rst_n),
        
        .data_ready(rx_ready),
        .data_valid(valid_rx),
        .cmd_data(rx_sys_data),
        .rd_en(rx_fifo_rd_en),
        
        .error_code(error_code),
        .enable(programming),
        .error_pulse(error_pulse)
    );
            
    // UART Instantiation
    logic baud = 1'b0;      // Signal toggling at baud rate
    logic rx_serial_ext;
    logic valid_tx = 1'b0;
    logic valid_rx;
    logic tx_serial_ext;
    logic o_tx_full;        // (Not used)
    logic done;

    logic [2:0] uart_rx_state; // (Not used)
    logic [2:0] uart_tx_state; // (Not used)
    logic [1:0] rx_fifo_fe;    // (Not used)
    logic [1:0] tx_fifo_fe;    // (Not used)

    UART #(.BAUD(BAUD_RATE), .CLK_F(CLK_FREQ_HZ)) uart (
        .clk(clk),
        .rst_n(rst_n),

        .i_test_mux(1'b0),  // 0: normal | 1: test
        .i_transmit(1'b1),
        .i_rx_serial_ext(rx_serial_ext),
        .i_rx_fifo_rd_en(rx_fifo_rd_en),
        .i_valid_tx(valid_tx),
        .i_tx_sys_data(8'h33),

        .o_tx_serial_ext(tx_serial_ext),
        .o_rx_sys_data(rx_sys_data),
        .o_valid_rx(valid_rx),
        .o_rx_ready(rx_ready),
        .o_done(done),
        .o_tx_full(o_tx_full),

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

    // Main test procedure
    logic [7:0] command_data [CMD_DEPTH][CMD_WIDTH];
    logic [7:0] num_commands;

    initial begin
        DEBUG_INFO("TB", "Testbench started!");
        programming = 1'b1;
        rx_serial_ext = 1'b1;
        #(CLK_PERIOD_NS);
        reset_n(rst_n);
        #(CLK_PERIOD_NS);

        // Reading default commands
        if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("READ_MEM", "Reading default commands");
        DEBUG_INFO("READ_MEM", $sformatf("Number of commands: %0d", num_commands));
        // Reading default commands starting from register 1 (skip index 0)
        for (int i = 0; i < CMD_DEPTH; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if_regs_inst.read_mem(command_data[i][j], $bits(if_regs_inst.addr)'(i*CMD_WIDTH + j + 1), test_fail); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("READ_MEM", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if (command_data[i][j] == 8'h0D) begin
                    DEBUG_INFOW($sformatf("\\r"));
                end else if (command_data[i][j] == 8'h0A) begin
                    DEBUG_INFOW($sformatf("\\n"));
                end else begin
                    DEBUG_INFOW($sformatf("%c", command_data[i][j]));
                end
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        // Write commands to memory via UART
        DEBUG_INFO("WRITE_MEM", "Writing commands to memory.");
        DEBUG_INFO("WRITE_MEM", "Clearing Command Memory.");
        send_uart_string(rx_serial_ext, "CR");
        if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("WRITE_MEM", $sformatf("Number of commands: %0d", num_commands));
        send_uart_string(rx_serial_ext, $sformatf("AT+TEST0%c\n", 8'h0D));
        send_uart_string(rx_serial_ext, $sformatf("AT+TEST01%c\n", 8'h0D));
        send_uart_string(rx_serial_ext, $sformatf("00 AT+TEST=1%c\n", 8'h0D));
        send_uart_string(rx_serial_ext, $sformatf("01 AT+TEST2=2%c\n", 8'h0D));
        send_uart_string(rx_serial_ext, $sformatf("22 AT+TEST2=2%c\n", 8'h0D));
        send_uart_string(rx_serial_ext, $sformatf("02 AT+VERY_LONG_COMMAND_OVER_32_CHARS=3%c\n", 8'h0D));
        if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("WRITE_MEM", $sformatf("Number of commands: %0d", num_commands));

        DEBUG_INFO("OVRW_CMD", "Overwriting commands.");
        if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("OVRW_CMD", $sformatf("Number of commands: %0d", num_commands));
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if_regs_inst.read_mem(command_data[i][j], $bits(if_regs_inst.addr)'(i*CMD_WIDTH + j + 1), test_fail); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("OVRW_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if (command_data[i][j] == 8'h0D) begin
                    DEBUG_INFOW($sformatf("\\r"));
                end else if (command_data[i][j] == 8'h0A) begin
                    DEBUG_INFOW($sformatf("\\n"));
                end else begin
                    DEBUG_INFOW($sformatf("%c", command_data[i][j]));
                end
            end
            DEBUG_INFOW($sformatf("\n"));
        end
        send_uart_string(rx_serial_ext, $sformatf("01 AT+TEST3=3%c\n", 8'h0D));
        if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("OVRW_CMD", $sformatf("Number of commands: %0d", num_commands));
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if_regs_inst.read_mem(command_data[i][j], $bits(if_regs_inst.addr)'(i*CMD_WIDTH + j + 1), test_fail); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("OVRW_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if (command_data[i][j] == 8'h0D) begin
                    DEBUG_INFOW($sformatf("\\r"));
                end else if (command_data[i][j] == 8'h0A) begin
                    DEBUG_INFOW($sformatf("\\n"));
                end else begin
                    DEBUG_INFOW($sformatf("%c", command_data[i][j]));
                end
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        DEBUG_INFO("ADD_CMD", "Adding commands.");
        for (int i = {($bits(i)-$bits(num_commands))'(1'b0),num_commands}; i < CMD_DEPTH-1; i++) begin
            send_uart_string(rx_serial_ext, $sformatf("%02d AT+TEST%0d=%0d%c\n", i, (i+2), (i+2), 8'h0D));
            if_regs_inst.read_mem(num_commands, 0, test_fail); // Using addr = 0, id = -1 just as a tag
            DEBUG_INFO("ADD_CMD", $sformatf("Number of commands: %0d", num_commands));
        end
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if_regs_inst.read_mem(command_data[i][j], $bits(if_regs_inst.addr)'(i*CMD_WIDTH + j + 1), test_fail); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("ADD_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                if (command_data[i][j] == 8'h0D) begin
                    DEBUG_INFOW($sformatf("\\r"));
                end else if (command_data[i][j] == 8'h0A) begin
                    DEBUG_INFOW($sformatf("\\n"));
                end else begin
                    DEBUG_INFOW($sformatf("%c", command_data[i][j]));
                end
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        // End simulation
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

    always_ff @( posedge clk ) begin : ERROR_MONITOR
        if (error_pulse) begin
            if (error_code == 2'd1) begin
               DEBUG_WARN("CMD_ERR", "Command too wide!");
            end else if (error_code == 2'd2) begin
               DEBUG_WARN("CMD_ERR", "Invalid address!");
            end else if (error_code == 2'd3) begin
               DEBUG_WARN("CMD_ERR", "Missing space!");
            end 
        end
    end

    // Dump waveform
    initial begin
        $dumpfile("waveform.fst");
        $dumpvars(0, cmd_mem_tb);
    end

endmodule
