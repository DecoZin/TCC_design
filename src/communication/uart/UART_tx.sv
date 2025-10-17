// ---------------------------------------------------------
// Module: UART TX
// Description: Module for UART tranceiver with 8 bits of data
// and 1 stop bit, no parity, and a baud rate generator.
// Code based on :
//  https://github.com/nandland/nandland/blob/master/uart/Verilog/source/UART_TX.v
// Author: André Lamego
// Date: 2025-04-08
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef UART_TX_SV
`define UART_TX_SV

module UART_tx #(
    parameter BAUD  = 9600,      // Default baud rate
    parameter CLK_F = 50_000_000   // Default clock frequency of 50MHz
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [7:0]  i_tx_data,
    input  logic        i_valid,
    output logic        o_tx_serial,
    output logic        o_busy,
    output logic        o_done,
    output logic [2:0]  t_state // Add this line to expose the state signal

);

    // Internal signal
    localparam int CLKS_PER_BIT = $rtoi(CLK_F / BAUD); // Calculate clock cycles per bit
    localparam int COUNTER_WIDTH = $clog2(CLKS_PER_BIT); // Dynamically calculate the counter width

// Enumerated state machine
    typedef enum logic [2:0] {
        IDLE        = 3'b000,
        START_BIT   = 3'b001,
        DATA_BITS   = 3'b010,
        STOP_BIT    = 3'b011,
        CLEANUP     = 3'b100
    } state_t;

    state_t state;
    assign t_state = state;

    // Internal registers
    logic [COUNTER_WIDTH-1:0] clk_counter; // Clock counter for baud rate generation
    logic [2:0] bit_index;    // Bit index for data bits
    logic [7:0] r_tx_data;    // Register to hold the data to be transmitted

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            o_tx_serial <= 1;                // Idle state for UART is high
            o_busy      <= 0;                // Not busy at reset
            o_done      <= 0;                // Not done at reset
            clk_counter <= 0;
            bit_index   <= 0;
            r_tx_data   <= 8'b0;
        end else begin
            o_done <= 0;                       // Default to not done
            case (state)
                IDLE: begin
                    o_tx_serial <= 1;               // Drive Line High for Idle
                    clk_counter <= 0;
                    bit_index   <= 0;

                    if (i_valid == 1) begin
                        r_tx_data <= i_tx_data;
                        o_busy    <= 1;
                        state     <= START_BIT;
                    end
                end // case: IDLE

                START_BIT: begin
                    o_tx_serial <= 0;               // Start bit is 0

                    // Wait CLKS_PER_BIT-1 clock cycles for start bit to finish
                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } < CLKS_PER_BIT-1) begin
                        clk_counter <= clk_counter + 1;
                    end else begin
                        clk_counter <= 0;
                        state       <= DATA_BITS;
                    end
                end // case: START_BIT

                DATA_BITS: begin
                    o_tx_serial <= r_tx_data[bit_index]; // Transmit current bit

                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } < CLKS_PER_BIT-1) begin
                        clk_counter <= clk_counter + 1;
                    end else begin
                        clk_counter <= 0;

                        // Check if we have sent out all bits
                        if (bit_index < 7) begin
                            bit_index <= bit_index + 1;
                        end else begin
                            bit_index <= 0;
                            state     <= STOP_BIT;
                        end
                    end
                end // case: DATA_BITS

                STOP_BIT: begin
                    o_tx_serial <= 1;               // Stop bit is 1

                    // Wait CLKS_PER_BIT-1 clock cycles for Stop bit to finish
                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } < CLKS_PER_BIT-1) begin
                        clk_counter <= clk_counter + 1;
                    end else begin
                        clk_counter <= 0;
                        o_done      <= 1;           // Transmission complete
                        o_busy      <= 0;           // Not busy anymore
                        state       <= CLEANUP;
                    end
                end // case: STOP_BIT

                CLEANUP: begin
                    state       <= IDLE;
                    clk_counter <= 0;
                    r_tx_data   <= 8'b0;
                end

                default: begin
                    state       <= IDLE;
                    o_tx_serial <= 1;
                    o_busy      <= 0;
                    o_done      <= 0;
                    clk_counter <= 0;
                    bit_index   <= 0;
                    r_tx_data   <= 8'b0;
                end
            endcase
        end
    end

endmodule

`endif // UART_TX_SV
