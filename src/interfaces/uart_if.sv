// ---------------------------------------------------------
// Module: UART Interface
// Description: Defines the interface for the UART
// Author: André Lamego
// Date: 2025-05-27
// ---------------------------------------------------------
`timescale 1ns / 1ps

`ifndef UART_SV
`define UART_SV

interface uart_rx_if (
        input logic clk,
        input logic rst_n
    );

    logic rd_en;            // Read enable signal
    logic ready;            // Data ready signal
    logic valid;            // Valid data arrived signal
    logic [7:0] data;       // Received data byte

    modport uart_rx (
        input  rd_en,
        output ready,
        output valid,
        output data
    );

    modport controller (
        output rd_en,
        input  ready,
        input  valid,
        input  data
    );

endinterface //uart_rx_if

interface uart_tx_if (
        input logic clk,
        input logic rst_n
    );

    logic done;             // Data sent signal
    logic full;             // TX FIFO full signal
    logic valid;            // Valid data to send signal
    logic [7:0] data;       // Data byte to send

    modport uart_tx (
        input  valid,
        input  data,
        output done,
        output full
    );

    modport controller (
        output valid,
        output data,
        input  done,
        input  full
    );

endinterface //uart_tx_if

`endif // UART_SV
