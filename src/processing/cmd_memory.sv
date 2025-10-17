// ---------------------------------------------------------------------
// Module: Command Memory
// Description: Module of memory for commands to send to the BLE module
// Author: André Lamego
// Date: 2025-05-12
// ---------------------------------------------------------------------
`timescale 1ns / 100ps

`ifndef CMD_MEMORY_SV
`define CMD_MEMORY_SV

module cmd_memory #(
    parameter CMD_WIDTH = 32,   // Number of bytes per command
    parameter CMD_DEPTH = 16    // Number of commands in memory

) (
    regs_if if_regs_inst,  // Interface instance for communication

    input logic clk,
    input logic rst_n,

    input logic enable,                 // Enable the programming of commands
    input logic data_ready,             // Read data is ready to be consumed
    input logic data_valid,             // FIFO has a valid data to be read
    input logic [7:0] cmd_data,         // Command data to write

    output logic rd_en,                 // Reads data from RX FIFO
    output logic [2:0] error_code,       // Error code for command memory
    output logic error_pulse            // Error Pulse for the error controller
);
    import cmd_mem_pkg::*;

    // Parameters
    localparam int DATA_WIDTH = 8;
    localparam int DATA_DEPTH = CMD_DEPTH*CMD_WIDTH;
    localparam int CMD_COUNTER_WIDTH = $clog2(CMD_DEPTH);
    localparam int CMD_ADDR_WIDTH = $clog2(DATA_DEPTH);  // total address width
    localparam int CMD_IDX = $clog2(CMD_WIDTH);


    // Internal signal
    logic [7:0] buffer[CMD_WIDTH];  // To store the message
    logic [CMD_COUNTER_WIDTH-1:0] cmd_counter;
    logic [CMD_IDX-1:0] index;      // Index within buffer
    logic [7:0] addr_ascii[1:0];    // To store two ASCII address chars
    logic [CMD_ADDR_WIDTH-1:0] target_addr;
    logic [CMD_IDX-1:0] cmd_index;

    // Interface instantiations
    regs_int_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link();

    // Register File instantiation
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(DATA_DEPTH)
    ) cmd_regs (
        .if_regs_inst(if_regs_inst.slave),
        .if_top_link(if_top_link.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    // Error code
    typedef enum logic [1:0] { 
        NO_ERR      = 2'd0,
        ERR_CMD_WDH = 2'd1,
        ERR_ADDR    = 2'd2,
        ERR_SPACE   = 2'd3
    } error_t;

    // FSM
    typedef enum logic [4:0] {
        RESET       = 5'd0,  // Load default values of commands to the memory
        IDLE        = 5'd1,  // Waiting for a new byte
        CLEAR       = 5'd2,  // Waiting for a new byte
        GET_ADDR1   = 5'd3,  // Get the first part of the address
        WAIT_ADDR2  = 5'd4,  // Wait for second ASCII address character
        GET_ADDR2   = 5'd5,  // Get the second part of the address
        WAIT_SPACE  = 5'd6,  // Wait for the space between adress and data 
        GET_SPACE   = 5'd7,  // Get the space between adress and data
        WAIT_DATA   = 5'd8,  // Wait for next string to be ready
        GET_DATA    = 5'd9,  // Buffering command string
        WAIT_LF     = 5'd10, // Buffering command string
        CHECK_LF    = 5'd11, // Saw '\r', waiting for '\n'
        STORE       = 5'd12  // Write buffer to memory

    } state_t;

    state_t state;
    state_t last_state;
    
    always_comb begin : BUFFER_REGI
        // Reserve address 0 for command counter
        // Commands start from address 1 onward
        if_top_link.regi[0] = cmd_counter; // Number of commands
        cmd_index = (addr_ascii[0] - "0") * 10 + (addr_ascii[1] - "0");
        target_addr = (cmd_index * CMD_WIDTH) + 1;
        if (state == RESET || ((state == IDLE && if_top_link.load_regs) && last_state == RESET)) begin
            assign_string_to_regi(HM10_CMD_AT,      1);
            assign_string_to_regi(HM10_CMD_NAME,   33);
            assign_string_to_regi(HM10_CMD_PIN,    65);
            assign_string_to_regi(HM10_CMD_BAUD,   97);
            assign_string_to_regi(HM10_CMD_ROLE,  129);
            assign_string_to_regi(HM10_CMD_RESET, 161);
            assign_string_to_regi(HM10_CMD_ADV,   193);
        end else if (if_top_link.load_regs) begin
            for (int i = 0; i < CMD_WIDTH; i++) begin
                if ((target_addr + i) < DATA_DEPTH) begin
                    if_top_link.regi[target_addr + i] = buffer[i];
                end 
            end
        end
    end 

    function void assign_string_to_regi(input string str, input int start_idx);
        for (int i = 0; i < str.len(); i++) begin
            if_top_link.regi[start_idx + i] = str[i];
        end
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if ( !rst_n ) begin
            index <= 4'd0;
            addr_ascii[0] <= "0";
            addr_ascii[1] <= "0";
            error_code <= NO_ERR;
            if_top_link.mode_mask = '1;  // All registers are Read-Only
            state <= RESET;
            last_state <= RESET;
        end else begin
            last_state <= state;
            if_top_link.load_regs <= 1'b0;
            rd_en <= 1'b0; // default: no read
            error_pulse <= 1'b0;
            index <= 4'd0; // Default if not in RECEIVING or CHECK_LF
            case ( state )
                RESET: begin
                    if_top_link.load_regs <= 1'b1;
                    cmd_counter <= 'd7;
                    state <= IDLE;
                end
                IDLE: begin
                    for (int i = 0; i < CMD_WIDTH; i++) begin
                        buffer[i] <= 8'h20;  // ' ' = ASCII 32 = 0x20
                    end
                    if (data_valid && enable) begin
                        rd_en <= 1'b1;
                        state <= GET_ADDR1;
                    end else begin
                        state <= IDLE;
                    end
                end
                GET_ADDR1: begin
                    if (data_ready) begin
                        state <= WAIT_ADDR2;
                        addr_ascii[0] <= cmd_data;
                    end else begin
                        state <= GET_ADDR1;
                    end
                end
                WAIT_ADDR2: begin
                    if ((addr_ascii[0] < "0") || (addr_ascii[0] > "9") &&
                        (addr_ascii[0] != "C")) begin
                            error_code <= ERR_ADDR;
                            error_pulse <= 1'b1;
                            state <= IDLE;    
                    end else if (data_valid) begin
                        state <= GET_ADDR2;
                        rd_en <= 1'b1;
                    end else begin
                        state <= WAIT_ADDR2;
                    end
                end
                GET_ADDR2: begin
                    if (data_ready) begin
                        state <= WAIT_SPACE;
                        addr_ascii[1] <= cmd_data;
                    end else begin
                        state <= GET_ADDR2;
                    end
                end
                WAIT_SPACE: begin
                    if (addr_ascii[0] == "C" && addr_ascii[1] == "R") begin
                        if_top_link.load_regs <= 1'b1;
                        cmd_counter <= 0;
                        state <= IDLE;
                    end else if (
                        (addr_ascii[0] < "0") || (addr_ascii[0] > "9") ||
                        (addr_ascii[1] < "0") || (addr_ascii[1] > "9")) begin
                        error_code <= ERR_ADDR;
                        error_pulse <= 1'b1;
                        state <= IDLE;
                    end else if (data_valid) begin
                        state <= GET_SPACE;
                        rd_en <= 1'b1;
                    end else begin
                        state <= WAIT_SPACE;
                    end
                end
                GET_SPACE: begin
                    if (data_ready) begin
                        if (cmd_data != 8'h20) begin
                            error_code <= ERR_SPACE;
                            error_pulse <= 1'b1;
                            state <= IDLE;
                        end else begin
                            state <= WAIT_DATA;
                        end
                    end else begin
                        state <= GET_SPACE;
                    end
                end
                WAIT_DATA: begin
                    index <= index;
                    // -1 because the first register is the command counter
                    // so the address of the last command is CMD_DEPTH-1
                    if (cmd_index >= (CMD_DEPTH-1)) begin
                        error_code <= ERR_ADDR;
                        error_pulse <= 1'b1;
                        state <= IDLE;
                    end else if (data_valid) begin
                        state <= GET_DATA;
                        rd_en <= 1'b1;
                    end else begin
                        state <= WAIT_DATA;
                    end
                end
                GET_DATA: begin
                    index <= index;
                     if (data_ready) begin
                        if (index >= (CMD_WIDTH-2)) begin
                            error_code <= ERR_CMD_WDH;
                            error_pulse <= 1'b1;
                            state <= IDLE;
                        // Checking for '\r'
                        end else if (cmd_data == 8'h0D) begin
                            buffer[index] <= cmd_data;
                            index <= index + 1;
                            state <= WAIT_LF;
                        end else begin
                            buffer[index] <= cmd_data;
                            index <= index + 1;
                            state <= WAIT_DATA;
                        end
                    end else begin
                        state <= GET_DATA;
                    end
                end
                WAIT_LF: begin
                    index <= index;
                    if (data_valid) begin
                        state <= CHECK_LF;
                        rd_en <= 1'b1;
                    end else begin
                        state <= WAIT_LF;
                    end
                end
                CHECK_LF: begin
                    index <= index;
                    if (data_ready) begin
                        if (index >= CMD_WIDTH) begin
                            error_code <= ERR_CMD_WDH;
                            error_pulse <= 1'b1;
                            state <= IDLE;
                        end else if (cmd_data == 8'h0A) begin
                            buffer[index] <= cmd_data;
                            index <= index + 1;
                            state <= STORE;
                        end else begin
                            buffer[index] <= cmd_data;
                            index <= index + 1;
                            state <= WAIT_DATA;
                        end
                    end else begin
                        state <= CHECK_LF;
                    end
                end
                STORE: begin
                    if_top_link.load_regs <= 1'b1;
                    if ((cmd_index+1) > cmd_counter) begin
                        cmd_counter <= cmd_counter + 1;
                    end
                    state <= IDLE;
                end
                default:
                    begin
                        state <= RESET;
                    end
            endcase
        end
    end

endmodule

`endif // CMD_MEMORY_SV
