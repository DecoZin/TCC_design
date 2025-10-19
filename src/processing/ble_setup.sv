// ------------------------------------------------
// Module: BLE Setup
// Description: BLE setup flow
// Author: André Lamego
// Date: 2025-03-07
// ------------------------------------------------
`timescale 1ns / 100ps

`ifndef BLE_SETUP_SV
`define BLE_SETUP_SV

module ble_setup #(
    parameter logic [23:0] ACK_TIMEOUT_US = 24'd2_100,  // 20 bits at 9600 baud
    parameter CMD_WIDTH = 32
) (
    // Interfaces
    regs_if if_regs_inst,       // Interface instance to get the commands
    tmr_if if_tmr_inst,         // Interface instance for the timer

    // Clock and Reset
    input  logic clk,           // Clock signal
    input  logic rst_n,         // Reset signal
    
    // UART RX
    input  logic ack_valid,     // Character of the acknowledge from RX
    input  logic ack_ready,     // Acknowledge byte ready to be read
    input  logic [7:0] ack_byte,// Character of the acknowledge from RX
    output logic get_ack_byte,  // Read enable to get the acknowledge byte from RX
    
    // Signals
    input  logic setting_up,    // Signal that indicates that is in setup state
    output logic fail,          // Error with AT+Commands
    output logic setup_done,    // Setup done
    
    // UART TX
    input  logic tx_done,       // Character of the command sent successfully
    input  logic tx_full,       // Flag of full TX FIFO
    output logic byte_ready,    // Character of the command to TX ready
    output logic [7:0] cmd_byte // Character of the command to TX
);
    import ble_setup_types_pkg::*;

    // Parameters
    localparam CMD_OFFSET = 1;
    localparam CMD_STRIDE = CMD_WIDTH;


    // Internal signals
    logic ok_found;                 // Flag to indicate if "OK" was found in the acknowledge
    logic error_found;              // Flag to indicate if "ERROR" was found in the acknowledge
    logic retry;                    // Flag to retry sending the command
    logic [7:0] ack_window [4:0];   // 5-byte sliding window
    logic [7:0] last_data;          // Last data read from the registers - used to detect \r\n
    logic [4:0] cmd_counter;        // Counter for the number of commands sent
    logic [4:0] byte_counter;       // Counter for the number of bytes sent in the current command
    logic [4:0] cmd_number;         // Number of commands in memory
    logic pre_evaluate;

    // FSM
     ble_setup_state_t state;

     always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fail <= 1'b0;                   // Reset fail signal
            if_regs_inst.read_en <= 1'b0;   // Disable read from registers
            if_regs_inst.addr <= '0;        // Start reading from address 0
            retry <= 1'b1;                  // Reset retry flag
            get_ack_byte <= 1'b0;           // Reset get acknowledge byte signal
            cmd_byte <= 8'd0;               // Reset command byte
            cmd_counter <= '0;              // Reset command counter
            byte_counter <= '0;             // Reset byte counter
            cmd_number <= '0;               // Reset command number 
            setup_done <= 1'b0;             // Reset setup done signal
            state <= IDLE;                  // Start in IDLE state
            byte_ready <= 1'b0;             // Default if not overwritten
            pre_evaluate <= 1'b0;
        end else begin
            if_regs_inst.read_en <= 1'b0;   // Default if not overwritten
            get_ack_byte <= 1'b0;           // Default if not overwritten
            byte_ready <= 1'b0;             // Default if not overwritten
            case (state)
                IDLE:
                begin
                    fail <= 1'b0;           // Reset fail signal
                    setup_done <= 1'b0;     // Reset setup done signal
                    cmd_counter <= '0;      // Reset command counter
                    byte_counter <= '0;     // Reset byte counter
                    pre_evaluate <= 1'b0;
                    if (setting_up) begin
                        if_regs_inst.read_en <= 1'b1;   // Enable read from registers
                        if_regs_inst.addr <= '0;        // Start reading from address 0
                        state <= GET_CMD_NUMBER;
                    end else begin
                        state <= IDLE; // Stay in IDLE if not setting up
                    end
                end 
                GET_CMD_NUMBER:
                begin
                    if (if_regs_inst.data_ready) begin
                        cmd_number <= if_regs_inst.read_data[4:0]; // Get the number of commands
                        if (if_regs_inst.read_data[4:0] == 0) begin
                            fail <= 1'b1;   // No commands to send, set fail
                            state <= IDLE;  // Go back to IDLE
                        end else begin
                            if_regs_inst.read_en <= 1'b1;
                            if_regs_inst.addr <= 'd1;
                            retry <= 1'b1;      // Set retry flag
                            state <= SEND_CMD;  // Move to sending commands
                        end
                    end
                end 
                SEND_CMD:
                begin
                    if (cmd_counter == cmd_number) begin
                        state <= IDLE;
                        setup_done <= 1'b1; // All commands sent, set setup done
                    end else if ({($bits(CMD_WIDTH)-$bits(byte_counter))'('0), byte_counter} > CMD_WIDTH) begin
                        fail <= 1'b1;   // Fail if byte counter exceeds limit
                        state <= IDLE;  // Go back to IDLE
                    end else if (if_regs_inst.data_ready) begin
                        if (if_regs_inst.read_data == 8'h0A && last_data == 8'h0D) begin
                            byte_ready <= 1'b1;
                            cmd_byte <= if_regs_inst.read_data;
                            byte_counter <= 5'd0;   // Reset byte counter after sending \n
                            state <= WAIT_ACK;
                        end else begin
                            byte_ready <= 1'b1;     // Indicate byte is ready to be sent
                            if_regs_inst.read_en <= 1'b1; // Read next byte
                            cmd_byte <= if_regs_inst.read_data;
                            byte_counter <= byte_counter + 1; // Increment byte counter
                            if_regs_inst.addr <= if_regs_inst.addr + 1; // Move to next byte
                            state <= SEND_CMD; // Stay in SEND_CMD state
                        end
                    end else begin
                        byte_ready <= 1'b0; // No byte ready if data not available
                        state <= SEND_CMD;                            
                    end
                end 
                WAIT_ACK: 
                begin
                    if (ack_valid) begin
                        get_ack_byte <= 1'b1;
                        pre_evaluate <= 1'b0;
                        state <= GET_ACK;
                    end else if (if_tmr_inst.done) begin
                        if (retry) begin
                            if_regs_inst.addr <= CMD_STRIDE * cmd_counter + CMD_OFFSET; // Reset address to the current command
                            if_regs_inst.read_en <= 1'b1;
                            state <= SEND_CMD;  // Retry sending the command                                
                            retry <= 1'b0;
                            byte_counter <= '0; // Reset byte counter
                        end else begin
                            fail <= 1'b1;
                            state <= IDLE;                                
                        end
                    end
                end
                GET_ACK:
                begin
                    if (ok_found || error_found) begin
                        state <= EVALUATE_ACK; // Move to evaluate acknowledge state
                    end else begin
                        if (ack_ready) begin
                            pre_evaluate <= 1'b1; // Delay one cycle for ack_window to update
                        end
                        if (pre_evaluate) begin 
                            pre_evaluate <= 1'b0; // Even with delay, did not find OK or ERROR
                            state <= WAIT_ACK; // Wait for next acknowledge byte
                        end
                    end
                end
                EVALUATE_ACK:
                begin
                    if (ok_found) begin
                        cmd_counter <= cmd_counter + 1; // Increment command counter
                        if_regs_inst.addr <= $bits(if_regs_inst.addr)'(CMD_STRIDE * ({($bits(CMD_OFFSET)-$bits(cmd_counter))'('0), cmd_counter} + 1) + CMD_OFFSET); // Move to next command
                        byte_counter <= '0; // Reset byte counter
                        retry <= 1'b1;      // Set retry flag
                        state <= FLUSH_RX;  // Go back to sending commands
                    end else if (error_found) begin
                        if (retry) begin
                            if_regs_inst.addr <= CMD_STRIDE * cmd_counter + CMD_OFFSET; // Reset address to the last command
                            state <= FLUSH_RX; // Retry sending the command
                            retry <= 1'b0;
                            byte_counter <= '0; // Reset byte counter
                        end else begin
                            fail <= 1'b1;   // Set fail if no retry left
                            state <= IDLE;  // Go back to IDLE
                        end
                    end else begin
                        fail <= 1'b1; // Set fail if no retry left
                        state <= IDLE; // Go back to IDLE
                    end
                end 
                FLUSH_RX:
                begin
                    get_ack_byte <= 1'b1; // Request to get the acknowledge byte until the RX FIFO is flushed
                    if (if_tmr_inst.done) begin
                        if_regs_inst.read_en <= 1'b1;
                        get_ack_byte <= 1'b0; // Request to get the acknowledge byte
                        state <= SEND_CMD; // Go back to sending commands
                    end 
                end 
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == SEND_CMD)
            ack_window <= '{default:8'd0};
        else if (ack_ready && state == GET_ACK) begin
            ack_window[0] <= ack_window[1];
            ack_window[1] <= ack_window[2];
            ack_window[2] <= ack_window[3];
            ack_window[3] <= ack_window[4];
            ack_window[4] <= ack_byte;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_data <= 8'd0;
        end else if (if_regs_inst.data_ready) begin
            last_data <= if_regs_inst.read_data;
        end
    end

    function automatic logic is_ok_found(input logic [7:0] w[4:0]);
        return (w[3] == "O" && w[4] == "K");
    endfunction

    function automatic logic is_error_found(input logic [7:0] w[4:0]);
        return (w[0] == "E" && w[1] == "R" && w[2] == "R" && w[3] == "O" && w[4] == "R");
    endfunction

    // Timer Handler
    assign if_tmr_inst.mode = 1'b0;    // Set timer to one-shot mode
    assign if_tmr_inst.time_count = ACK_TIMEOUT_US;
    
    always_latch begin
        if_tmr_inst.enable = 1'b0;
        case (state)
            IDLE: begin
                if_tmr_inst.clear = 1'b1;   // Clear timer
            end
            WAIT_ACK: begin
                if_tmr_inst.clear = (tx_done || ack_ready) ? 1'b1 : 1'b0; // Release clear if UART is idle
                if_tmr_inst.enable = 1'b1;
                ok_found = 1'b0; // Reset ok_found flag
                error_found = 1'b0; // Reset error_found flag
            end
            GET_ACK: begin
                if_tmr_inst.clear = 1'b1; // Clear timer
                ok_found    = is_ok_found(ack_window);
                error_found = is_error_found(ack_window);
             end
            FLUSH_RX: begin
                if_tmr_inst.clear = (ack_ready || ack_valid) ? 1'b1 : 1'b0; // Clear timer if UART RX is idle
                if_tmr_inst.enable = 1'b1; // Enable timer
            end
            default: begin
                if_tmr_inst.clear = 1'b1; // Clear timer
                if_tmr_inst.enable = 1'b0;
            end
        endcase
    end
    
     
endmodule

`endif // BLE_SETUP_SV
