// ---------------------------------------------------------
// Module: UART
// Description: Module for UART tranceiver with 8 bits of data
// and 1 stop bit, no parity, and a baud rate generator.
// Code based on :
//  https://github.com/nandland/nandland/blob/master/uart/Verilog/source
// Author: André Lamego
// Date: 2025-04-08
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef UART_SV
`define UART_SV

module UART #(
    parameter BAUD  = 9600,       // Default baud rate
    parameter CLK_F = 50_000_000,   // Default clock frequency of 50MHz
    parameter RX_FIFO_DEPTH = 64, // RX FIFO depth
    parameter TX_FIFO_DEPTH = 64  // TX FIFO depth
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        i_test_mux,         // External MUX controller
    input  logic        i_transmit,         // Transmit data from RX to TX
    input  logic        i_rx_serial_ext,    // External RX serial data
    input  logic        i_rx_fifo_rd_en,    // Enable to read from RX FIFO
    input  logic        i_valid_tx,         // Valid signal for TX FIFO 
    input  logic [7:0]  i_tx_sys_data,      // Data from system to TX FIFO
    output logic [7:0]  o_rx_sys_data,      // Data to system from RX FIFO
    output logic        o_tx_serial_ext,    // External TX serial data
    output logic        o_valid_rx,         // Valid signal from RX FIFO
    output logic        o_done,
    output logic        o_rx_ready,
    output logic        o_tx_full,          // TX FIFO full signal
    output logic [2:0]  uart_rx_state,      // TEST SIGNALS
    output logic [2:0]  uart_tx_state,      // TEST SIGNALS
    output logic [1:0]  rx_fifo_fe,         // TEST SIGNALS
    output logic [1:0]  tx_fifo_fe          // TEST SIGNALS
);

    // Internal signals for FIFOs
    logic [7:0] rx_fifo_wr_data;            // Data to RX FIFO
    logic [7:0] tx_fifo_wr_data;            // Data to TX FIFO
    logic [7:0] tx_fifo_rd_data;            // Data from TX FIFO
    logic       rx_fifo_wr_en;              // RX FIFO write enable
    logic       rx_fifo_rd_en;              // RX FIFO read enable
    logic       tx_fifo_wr_en;              // TX FIFO write enable
    wire        tx_fifo_rd_en;              // TX FIFO read enable
    logic       rx_fifo_full, rx_fifo_empty;
    logic       tx_fifo_full, tx_fifo_empty;


    assign o_tx_full = tx_fifo_full; // Output TX FIFO full signal

    // TEST SIGNALS
    logic valid, tx_valid;
    logic [7:0] data;
    logic tx_fifo_wr_en_q;
    logic transmit;

    // ---------------------------------------------------------
    // Instantiate the UART RX module
    UART_rx #(.BAUD(BAUD), .CLK_F(CLK_F)) uart_rx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .i_rx_serial(i_rx_serial_ext),  // External RX serial data
        .o_valid(rx_fifo_wr_en),    // RX FIFO write enable
        .o_rx_data(rx_fifo_wr_data),   // RX FIFO data
        .t_state(uart_rx_state) // Connect state signal
    );

    // ---------------------------------------------------------
    // Instantiate the RX FIFO
    fifo #(
        .DEPTH(RX_FIFO_DEPTH)
    ) rx_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .i_wr_en(rx_fifo_wr_en),        // Write when UART_rx has valid data
        .i_wr_data(rx_fifo_wr_data),    // Data from UART_rx
        .i_rd_en(rx_fifo_rd_en),        // Read when system requests data
        .o_rd_data(o_rx_sys_data),      // Data to system
        .o_full(rx_fifo_full),          // Not used externally for RX
        .o_empty(rx_fifo_empty),
        .o_ready_pulse(o_rx_ready)
    );

    // Synchronize the asynchronous i_test_mux and i_transmit signals to the clock domain
    logic sync_test_mux, sync_transmit;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_test_mux <= 0;
            sync_transmit <= 0;
        end else begin
            sync_test_mux <= i_test_mux;
            sync_transmit <= i_transmit;
        end
    end

    // MUX For Looping back the RX data to TX FIFO
    assign rx_fifo_rd_en = sync_test_mux ? !tx_fifo_full : i_rx_fifo_rd_en;
    assign rx_fifo_fe = {rx_fifo_full, rx_fifo_empty}; 
    assign o_valid_rx = ~rx_fifo_empty;

    // ---------------------------------------------------------
    // Instantiate the UART TX module

    UART_tx #(.BAUD(BAUD), .CLK_F(CLK_F)) uart_tx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .i_tx_data(tx_fifo_rd_data),    // Data from TX FIFO
        .i_valid(tx_valid),             // Valid when TX FIFO is not empty
        .i_data_ready(tx_ready_pulse),  // Data ready to be used
        .o_tx_rd_en(tx_fifo_rd_en),     // TX FIFO read enable
        .o_tx_serial(o_tx_serial_ext),
        .o_done(o_done),
        .t_state(uart_tx_state)         // Connect state signal
    );

    // ---------------------------------------------------------
    // Instantiate the TX FIFO
    logic tx_ready_pulse;

    fifo #(
        .WIDTH(8),
        .DEPTH(TX_FIFO_DEPTH)
    ) tx_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .i_wr_en(tx_fifo_wr_en),        // Write when system provides data
        .i_wr_data(tx_fifo_wr_data),    // Data from system
        .i_rd_en(tx_fifo_rd_en),        // Read when UART_tx is ready
        .o_rd_data(tx_fifo_rd_data),    // Data to UART_tx
        .o_full(tx_fifo_full),
        .o_empty(tx_fifo_empty),
        .o_ready_pulse(tx_ready_pulse)
    );

    // MUX For Looping back the RX data to TX FIFO
    assign tx_fifo_wr_data = sync_test_mux ? o_rx_sys_data : i_tx_sys_data;
    assign tx_fifo_fe = {tx_fifo_full, tx_fifo_empty};
    assign valid = sync_test_mux ? !rx_fifo_empty : i_valid_tx;
    assign transmit = sync_test_mux ? sync_transmit : 1'b1; 
    assign tx_fifo_wr_en = valid && (!tx_fifo_full && transmit);
    assign tx_valid = !tx_fifo_empty;

endmodule

`endif // UART_SV
