// ---------------------------------------------------------
// Module: Command Memory Testbench
// Description: Testbench for the Command Memory module
// Author: André Lamego
// Date: 2025-05-08
// ---------------------------------------------------------
`timescale 1ns / 100ps

module cmd_mem_tb;
    import tb_tasks_pkg::*;	

    logic clk   = 0;
    logic rst_n = 1;

    localparam real CLK_FREQ = 50_000_000; // Clock frequency in Hz
    localparam real CLK_PERIOD = 1e9 / CLK_FREQ; // in ns
    localparam real BAUD_RATE = 9600; // Baud rate
    localparam real BIT_PERIOD = 1e9 / BAUD_RATE; // = 104166.66 ns (when timescale is 1ns)
    localparam CMD_WIDTH = 32;
    localparam CMD_DEPTH = 16;
    localparam DATA_WIDTH = 8;
    localparam CMD_DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;

    // Interface instantiation
    typedef virtual regs_if#(
        CMD_DATA_DEPTH,
        DATA_WIDTH
    ) regs_if_t;
        
    regs_if #(
        .DATA_DEPTH(CMD_DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_regs_inst(
        .clk(clk),
        .rst_n(rst_n)
    );
    regs_if_t vif = if_regs_inst;

    // Task class instantiation
    tb_tasks #(CMD_DATA_DEPTH, DATA_WIDTH) tasks;
    
    initial begin
        tasks = new(vif);
    end

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

    // Instantiate the UART module
    logic sel = 0;
    logic rx_serial_ext;
    logic valid_tx;
    logic tx_serial_ext;
    logic done;

    UART #(.BAUD(BAUD_RATE), .CLK_F(CLK_FREQ)) uart0 (
        .clk(clk),
        .rst_n(rst_n),
        .i_test_mux(sel),
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
        .uart_rx_state(),
        .uart_tx_state(),
        .rx_fifo_fe(),
        .tx_fifo_fe()
    );

    // Generate the clocks
    initial fork
        tasks.generate_clock(clk, CLK_PERIOD / 2);
    join_none

    logic baud;
    initial fork
        tasks.generate_clock(baud, BIT_PERIOD);
    join_none

    // Main test procedure
    logic [7:0] command_data [CMD_DEPTH][CMD_WIDTH];
    logic [7:0] num_commands;

    initial begin
        DEBUG_INFO("TB", "Testbench started!");
        programming = 1'b1;
        rx_serial_ext = 1'b1;
        #10;
        tasks.reset_n(rst_n);
        #10;

        // Reading default commands
        tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
        $write("\n");
        DEBUG_INFO("READ_MEM", "Reading default commands");
        DEBUG_INFO("READ_MEM", $sformatf("Number of commands: %0d", num_commands));
        // Reading default commands starting from register 1 (skip index 0)
        for (int i = 0; i < CMD_DEPTH; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                tasks.read_mem(command_data[i][j], i * CMD_WIDTH + j + 1, i); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("READ_MEM", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                DEBUG_INFOW($sformatf("%c", command_data[i][j]));
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        // Write commands to memory via UART
        $write("\n");
        DEBUG_INFO("WRITE_MEM", "Writing commands to memory.");
        DEBUG_INFO("WRITE_MEM", "Clearing Command Memory.");
        tasks.send_uart_string(rx_serial_ext, "CR");
        tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("WRITE_MEM", $sformatf("Number of commands: %0d", num_commands));
        tasks.send_uart_string(rx_serial_ext, $sformatf("AT+TEST0%c\n", 8'h0D));
        tasks.send_uart_string(rx_serial_ext, $sformatf("AT+TEST01%c\n", 8'h0D));
        tasks.send_uart_string(rx_serial_ext, $sformatf("00 AT+TEST=1%c\n", 8'h0D));
        tasks.send_uart_string(rx_serial_ext, $sformatf("01 AT+TEST2=2%c\n", 8'h0D));
        tasks.send_uart_string(rx_serial_ext, $sformatf("22 AT+TEST2=2%c\n", 8'h0D));
        tasks.send_uart_string(rx_serial_ext, $sformatf("02 AT+VERY_LONG_COMMAND_OVER_32_CHARS=3%c\n", 8'h0D));
        tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("WRITE_MEM", $sformatf("Number of commands: %0d", num_commands));

        $write("\n");
        DEBUG_INFO("OVRW_CMD", "Overwriting commands.");
        tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("OVRW_CMD", $sformatf("Number of commands: %0d", num_commands));
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                tasks.read_mem(command_data[i][j], i * CMD_WIDTH + j + 1, i); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("OVRW_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                DEBUG_INFOW($sformatf("%c", command_data[i][j]));
            end
            DEBUG_INFOW($sformatf("\n"));
        end
        tasks.send_uart_string(rx_serial_ext, $sformatf("01 AT+TEST3=3%c\n", 8'h0D));
        tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
        DEBUG_INFO("OVRW_CMD", $sformatf("Number of commands: %0d", num_commands));
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                tasks.read_mem(command_data[i][j], i * CMD_WIDTH + j + 1, i); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("OVRW_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                DEBUG_INFOW($sformatf("%c", command_data[i][j]));
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        $write("\n");
        DEBUG_INFO("ADD_CMD", "Adding commands.");
        for (int i = num_commands; i < CMD_DEPTH-1; i++) begin
            tasks.send_uart_string(rx_serial_ext, $sformatf("%02d AT+TEST%0d=%0d%c\n", i, (i+2), (i+2), 8'h0D));
            tasks.read_mem(num_commands, 0, -1); // Using addr = 0, id = -1 just as a tag
            DEBUG_INFO("ADD_CMD", $sformatf("Number of commands: %0d", num_commands));
        end
        for (int i = 0; i < num_commands; i++) begin
            for (int j = 0; j < CMD_WIDTH; j++) begin
                tasks.read_mem(command_data[i][j], i * CMD_WIDTH + j + 1, i); // +1 to offset starting at addr=1
            end

            DEBUG_INFOWT("ADD_CMD", $sformatf("Command %0d: ", i));
            for (int j = 0; j < CMD_WIDTH; j++) begin
                DEBUG_INFOW($sformatf("%c", command_data[i][j]));
            end
            DEBUG_INFOW($sformatf("\n"));
        end

        // End simulation
        #20;
        $write("\n");
        DEBUG_INFO("TB", "Testbench finished.");
        $finish;        
    end

    always_ff @( posedge clk ) begin : ERROR_MONITOR
        if (error_pulse) begin
            if (error_code == 1) begin
               DEBUG_ERR("CMD_ERR", "Command too wide!");
            end else if (error_code == 2) begin
               DEBUG_ERR("CMD_ERR", "Invalid address!");
            end else if (error_code == 3) begin
               DEBUG_ERR("CMD_ERR", "Missing space!");
            end else if (error_code == 4) begin
               DEBUG_ERR("CMD_ERR", "Memory full!");
            end
        end
    end

    // Dump waveform
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, cmd_mem_tb);
    end

    initial begin
        #10_000_000_000;
        DEBUG_ERR("TB", "Global timeout hit! Simulation stuck.");
        $finish;
    end    

endmodule
