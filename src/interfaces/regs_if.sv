// ---------------------------------------------------------
// Module: Registers Memory Interface
// Description: Defines the interface for the registers memory
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef REGS_IF_SV
`define REGS_IF_SV

interface regs_if #(
    parameter DATA_DEPTH = 16, // Number of registers
    parameter DATA_WIDTH = 8   // Data width
)(
    input logic clk,
    input logic rst_n
);

    localparam ADDR_WIDTH = $clog2(DATA_DEPTH); // Address width

    logic [DATA_WIDTH-1:0] read_data;  
    logic data_ready;
    logic write_done;
    logic write_en;
    logic read_en;
    logic [ADDR_WIDTH-1:0] addr;
    logic [DATA_WIDTH-1:0] write_data;

    // =========================================================
    // TASKS inside the interface — Verilator-compatible
    // =========================================================

    // Write to memory-mapped register
    task automatic write_mem(
        input logic [ADDR_WIDTH-1:0] addr_val,
        input logic [DATA_WIDTH-1:0] data_val
    );
        write_en   = 1;
        read_en    = 0;
        addr       = addr_val;
        write_data = data_val;
        @(posedge clk);
        #1;
        write_en = 0;
        @(posedge clk);
    endtask

    // Read from memory-mapped register
    task automatic read_mem(
        input  logic [ADDR_WIDTH-1:0] addr_val,
        output logic [DATA_WIDTH-1:0] data_out
    );
        int timeout = 1000;

        addr     = addr_val;
        read_en  = 1;
        write_en = 0;
        @(posedge clk);
        #1;

        while (data_ready == 0 && timeout > 0) begin
            @(posedge clk);
            timeout--;
        end

        if (timeout == 0) begin
            $display("\033[1;31m[ERROR][MEM] Timeout waiting for data_ready at address %0d!\033[0m", addr_val);
            $finish;
        end

        data_out = read_data;
        read_en  = 0;
        @(posedge clk);
    endtask

    // =========================================================
    // Modports
    // =========================================================
    modport master (
        input  read_data,
        input  data_ready,
        input  write_done,
        output write_en,
        output read_en,
        output addr,
        output write_data
    );

    modport slave (
        output read_data,
        output data_ready,
        output write_done,
        input  write_en,
        input  read_en,
        input  addr,
        input  write_data
    );

endinterface

`endif // REGS_IF_SV
