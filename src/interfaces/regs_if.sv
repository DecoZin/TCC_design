// ---------------------------------------------------------
// Module: Registers Memory Interface
// Description: Defines the interface for the registers memory
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 1ps

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
    logic [DATA_WIDTH-1:0] read_data;  // Data to read
    logic data_ready;  // Read ack signal
    logic write_done; // Write ack signal
    logic write_en; // Write enable signal
    logic read_en;  // Read enable signal
    logic [ADDR_WIDTH-1:0] addr; // Address to write/read data
    logic [DATA_WIDTH-1:0] write_data; // Data to write

    modport master (
        input read_data,
        input data_ready,
        input write_done,
        output write_en,
        output read_en,
        output addr,
        output write_data
    );

    modport slave (
        output read_data,
        output data_ready,
        output write_done,
        input write_en,
        input read_en,
        input addr,
        input write_data
    );

endinterface //regs_if

interface regs_int_if #(
    parameter DATA_WIDTH = 8,    // Data width for each register
    parameter DATA_DEPTH = 16    // Total number of registers
)(
    input logic clk,
    input logic rst_n
);

    // Define a type for register data
    typedef logic [DATA_WIDTH-1:0] reg_data_t;
    typedef reg_data_t reg_file_t[DATA_DEPTH];  // Array of registers

    // Registers and control signals
    logic load_regs; // Transfer data from REGI to the registers 
    reg_file_t regi;      // Initial values for the registers (input from master)
    reg_file_t rego;      // Output register values (sent to master)
    logic [DATA_DEPTH-1:0] mode_mask;  // Control mask for read/write operations (1 = read-only, 0 = read/write)

    modport master (
        output regi,       // Master writes to registers (outputs initial values)
        output load_regs,  // Master indicates register assignment
        output mode_mask,   // Master controls read/write mode (input control signal)
        input  rego       // Master reads from registers (inputs output values)
    );

    modport slave (
        input  regi,       // Slave receives register data (input values from master)
        input  load_regs,  // Slave assigns registers based on regi
        input  mode_mask,   // Slave provides read/write control (outputs control signals)
        output rego       // Slave provides register outputs (outputs values to master)
    );

endinterface


`endif // REGS_IF_SV
