// ------------------------------------------------
// Module: BLE Setup
// Description: BLE setup flow
// Author: André Lamego
// Date: 2025-03-07
// ------------------------------------------------
`ifndef BLE_SETUP_SV
`define BLE_SETUP_SV

module ble_setup (
    // Interfaces
    regs_if if_regs_inst,       // Interface instance to get the commands
    tmr_if if_tmr,              // Interface instance for the timer

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
    input  logic tx_full        // Flag of full TX FIFO
    output logic byte_ready,    // Character of the command to TX ready
    output logic [7:0] cmd_byte // Character of the command to TX
);

    // Parameters
    localparam MAX_CMD_LEN = 32;
    localparam CMD_OFFSET = 1;
    localparam CMD_STRIDE = MAX_CMD_LEN;


    // Internal signals
    logic ok_found;                 // Flag to indicate if "OK" was found in the acknowledge
    logic error_found;              // Flag to indicate if "ERROR" was found in the acknowledge
    logic retry;                    // Flag to retry sending the command
    logic request_ack_byte;         // Request to get the acknowledge byte
    logic store_ack;                // Flag to store the acknowledge byte in the window
    logic [7:0] ack_window [4:0];   // 5-byte sliding window
    logic [7:0] last_data;          // Last data read from the registers - used to detect \r\n
    logic [4:0] cmd_counter;        // Counter for the number of commands sent
    logic [4:0] byte_counter;       // Counter for the number of bytes sent in the current command
    logic [4:0] cmd_number;         // Number of commands in memory

    // FSM
    typedef enum logic [3:0] { 
        IDLE            = 'd1, // Not in setup state
        GET_CMD_NUMBER  = 'd2, // Get the number of commands in memory
        SEND_CMD        = 'd3, // Send the command byte/byte to TX FIFO until \r\n
        WAIT_ACK        = 'd4, // Wait for the acknowledge byte
        GET_ACK         = 'd5, // Get the acknowledge byte and put it in the buffer
        EVALUATE_ACK    = 'd6, // Evaluate the acknowledge has OK in the first 5 bytes or has a ERROR
        FLUSH_RX        = 'd7  // Flush the RX FIFO after knowing the result if the ack is longer than 5 bytes "OK+..."
     } state_t;

     state_t state;


     always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fail <= 1'b0;                   // Reset fail signal
            if_regs_inst.read_en <= 1'b0;   // Disable read from registers
            if_regs_inst.addr <= '0;        // Start reading from address 0
            retry <= 1'b1;                  // Reset retry flag
            request_ack_byte <= 1'b1;       // Request to get the acknowledge byte
            get_ack_byte <= 1'b0;           // Reset get acknowledge byte signal
            store_ack <= 1'b0;              // Reset store acknowledge byte signal
            cmd_byte <= 8'd0;               // Reset command byte
            cmd_counter <= '0;              // Reset command counter
            byte_counter <= '0;             // Reset byte counter
            cmd_number <= '0;               // Reset command number 
            setup_done <= 1'b0;             // Reset setup done signal
            state <= IDLE;                  // Start in IDLE state
            byte_ready <= 1'b0;             // Default if not overwritten
        end else begin
            if_regs_inst.read_en <= 1'b0;   // Default if not overwritten
            get_ack_byte <= 1'b0;           // Default if not overwritten
            store_ack <= 1'b0;              // Default if not overwritten
            byte_ready <= 1'b0;             // Default if not overwritten
            case (state)
                IDLE:
                begin
                    request_ack_byte <= 1'b1; // Request to get the acknowledge byte
                    fail <= 1'b0;           // Reset fail signal
                    setup_done <= 1'b0;     // Reset setup done signal
                    cmd_counter <= '0;      // Reset command counter
                    byte_counter <= '0;     // Reset byte counter
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
                        cmd_number <= if_regs_inst.read_data; // Get the number of commands
                        if (cmd_number == 0) begin
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
                    end else if (byte_counter > MAX_CMD_LEN) begin
                        fail <= 1'b1;   // Fail if byte counter exceeds limit
                        state <= IDLE;  // Go back to IDLE
                    end else if (if_regs_inst.data_ready) begin
                        if (if_regs_inst.read_data == 8'h0A && last_data == 8'h0D) begin
                            byte_ready <= 1'b1;
                            cmd_byte <= if_regs_inst.read_data;
                            byte_counter <= 5'd0;   // Reset byte counter after sending \n
                            if_tmr.clear <= 1'b1;   // Clear timer
                            if_tmr.mode <= 1'b0;    // Set timer to one-shot mode
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
                    if (if_tmr.done) begin
                        if (ack_valid) begin
                            state <= GET_ACK;
                        end else begin
                            if (retry) begin
                                if_regs_inst.addr <= CMD_STRIDE * cmd_counter + CMD_OFFSET; // Reset address to the current command
                                state <= SEND_CMD;  // Retry sending the command                                
                                retry <= 1'b0;
                                byte_counter <= '0; // Reset byte counter
                            end else begin
                                fail <= 1'b1;
                                state <= IDLE;                                
                            end
                        end
                    end
                end
                GET_ACK:
                begin
                    if (ok_found || error_found) begin
                        state <= EVALUATE_ACK; // Move to evaluate acknowledge state
                    end else if (ack_valid) begin
                        if (request_ack_byte) begin
                            get_ack_byte <= 1'b1;
                            request_ack_byte <= 1'b0;
                        end else begin
                            if (ack_ready) begin
                                store_ack <= 1'b1; // Acknowledge byte is ready
                                request_ack_byte <= 1'b1; // Request next acknowledge byte
                            end
                        end
                    end else if (!ack_valid && ack_ready) begin
                        // Last byte of acknowledge received will arrive when valid is low
                        store_ack <= 1'b1; // Acknowledge byte is ready
                        request_ack_byte <= 1'b1; // Request next acknowledge byte
                    end else begin
                        state <= EVALUATE_ACK; // Move to evaluate acknowledge state
                    end
                end 
                EVALUATE_ACK:
                begin
                    if (ok_found) begin
                        cmd_counter <= cmd_counter + 1; // Increment command counter
                        if_regs_inst.addr <= CMD_STRIDE * (cmd_counter + 1) + CMD_OFFSET; // Move to next command
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
                    if (ack_valid) begin
                        get_ack_byte <= 1'b1; // Request to get the acknowledge byte
                    end else begin
                        state <= SEND_CMD; // Go back to sending commands
                    end
                end 
                default: 
            endcase
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ack_window <= '{default:8'd0};
        else if (store_ack) begin
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

    assign if_tmr.time_count = 24'd250_000; // 250ms timeout
    always_comb begin
        case (state)
            IDLE: begin
                if_tmr.enable = 1'b0;
            end
            GET_CMD_NUMBER: begin
                if_tmr.enable = 1'b0;
            end
            SEND_CMD: begin
                if_tmr.enable = 1'b0;
            end
            WAIT_ACK: begin
                
                if_tmr.enable = 1'b1;
                ok_found = 1'b0; // Reset ok_found flag
                error_found = 1'b0; // Reset error_found flag
            end
            GET_ACK: begin
                if_tmr.enable = 1'b0;
                ok_found    = is_ok_found(ack_window);
                error_found = is_error_found(ack_window);
             end
            EVALUATE_ACK: begin
                if_tmr.enable = 1'b0;
            end
            FLUSH_RX: begin
                if_tmr.enable = 1'b0;
            end
            default: begin
                if_tmr.enable = 1'b0;
                ok_found = 1'b0; // Reset ok_found flag
                error_found = 1'b0; // Reset error_found flag
            end
        endcase
    end
    
     
endmodule

`endif // BLE_SETUP_SV
