// ---------------------------------------------------------
// Module: UART RX
// Description: Module for UART receiver with 8 bits of data
// and 1 stop bit, no parity, and a baud rate generator.
// Code based on :
//  https://github.com/nandland/nandland/blob/master/uart/Verilog/source/UART_RX.v
// Author: André Lamego
// Date: 2025-04-08
// ---------------------------------------------------------
`timescale 1ns / 1ps

`ifndef UART_RX_SV
`define UART_RX_SV

module UART_rx #(
    parameter BAUD  = 9600,      // Default baud rate
    parameter CLK_F = 50_000_000 // Default clock frequency of 50MHz
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        i_rx_serial,
    output logic        o_valid,
    output logic [7:0]  o_rx_data,
    output logic [2:0]  t_state // Exposing the state signal for debugging purposes
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
    logic [2:0]               bit_index;   // Bit index for data bits
    logic [7:0]               shift_reg;   // Shift register for received data

    // State Machine Controller
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            o_valid     <= 1'b0;
            clk_counter <= '0;
            bit_index   <= 3'b0;
            shift_reg   <= 8'b0;
        end else begin
            case (state)
                IDLE: begin
                    o_valid     <= 1'b0;
                    clk_counter <= '0;
                    bit_index   <= 3'b0;

                    if (i_rx_serial == 1'b0) // Start bit detected
                        state <= START_BIT;
                end

                START_BIT: begin
                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } == (CLKS_PER_BIT - 1) / 2) begin
                        if (i_rx_serial == 1'b0) begin
                            clk_counter <= '0; // Reset counter
                            state       <= DATA_BITS;
                        end else begin
                            state <= IDLE; // False start bit, potentially noisy input
                        end
                    end else begin
                        clk_counter <= clk_counter + 1;
                    end
                end

                DATA_BITS: begin
                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } == CLKS_PER_BIT - 1) begin
                        clk_counter          <= '0;
                        shift_reg[bit_index] <= i_rx_serial; // Store received bit

                        if (bit_index == 7) begin
                            state <= STOP_BIT; // All bits received
                        end else begin
                            bit_index <= bit_index + 1;
                        end
                    end else begin
                        clk_counter <= clk_counter + 1;
                    end
                end

                STOP_BIT: begin
                    if ({ ($bits(CLKS_PER_BIT)-$bits(clk_counter ))'('0),clk_counter } == CLKS_PER_BIT - 1) begin
                        if (i_rx_serial == 1'b1) begin // Validate stop bit
                            o_valid <= 1'b1; // Data is valid
                            o_rx_data <= shift_reg; // Output received data
                            state <= CLEANUP;
                        end else begin
                            state <= IDLE; // Stop bit error
                        end
                        clk_counter <= '0;
                    end else begin
                        clk_counter <= clk_counter + 1;
                    end
                end

                CLEANUP: begin
                    o_valid <= 1'b0; // Clear valid signal
                    state   <= IDLE; // Return to IDLE state
                end

                default: begin
                    state       <= IDLE;
                    o_valid     <= 1'b0;
                    clk_counter <= '0;
                    bit_index   <= 3'b0;
                    shift_reg   <= 8'b0;
                end
            endcase
        end
    end

endmodule

`endif // UART_RX_SV
