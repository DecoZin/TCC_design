// ---------------------------------------------------------
// Module: Timer Interface
// Description: Defines the interface for the timer
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef TMR_IF_SV
`define TMR_IF_SV

interface tmr_if (
        input logic clk,
        input logic rst_n
    );

    logic        enable;       // Enable to start counting
    logic        mode;         // 0: One-shot; 1: Auto-reload
    logic [23:0] time_count;   // Time to count in µs up to 10s
    logic        clear;        // Clear the counter
    logic        done;         // Signal after counting

    modport controller (
        input  done,
        output enable,
        output mode,
        output time_count,
        output clear
    );

    modport timer (
        output done,
        input  enable,
        input  mode,
        input  time_count,
        input  clear
    );

endinterface //tmr_if

`endif // TMR_IF_SV
