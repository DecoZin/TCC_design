// ---------------------------------------------------------
// Module: Register File
// Description: Module of memory 
// Author: André Lamego
// Date: 2025-04-30
// ---------------------------------------------------------
`timescale 1ns / 1ps

`ifndef REGISTER_FILE_SV
`define REGISTER_FILE_SV

module register_file #(
    parameter DATA_WIDTH  = 8,
    parameter DATA_DEPTH = 16  // Number of registers
) (
    regs_if if_regs_inst,  // Interface instance for communication
    regs_int_if if_top_link,  // Interface for register access with the top module
    input  logic clk,      // Clock signal
    input  logic rst_n     // Reset signal (active low)
    );

    localparam ADDR_WIDTH = $clog2(DATA_DEPTH);
            
    // Internal register array
    logic [DATA_WIDTH-1:0] regs[DATA_DEPTH];

    // Sequential logic for register operations
    always_ff @(posedge clk or negedge rst_n) begin
        if_regs_inst.data_ready <= 1'b0;
        if_regs_inst.write_done <= 1'b0;
        if (!rst_n || if_top_link.load_regs) begin
            // Initialize registers with values from if_top_link.regi on reset
            for (int i = 0; i < DATA_DEPTH; i++) begin
                regs[i] <= if_top_link.regi[i];  // Assign initial register values
            end
        end else if (if_regs_inst.write_en) begin
            // Write operation: check if register is not read-only
            if (!if_top_link.mode_mask[if_regs_inst.addr]) begin
                if_regs_inst.write_done <= 1'b1;
                regs[if_regs_inst.addr] <= if_regs_inst.write_data;
            end
        end else if (if_regs_inst.read_en) begin
            // Read operation: output register data
            if_regs_inst.data_ready <= 1'b1;
            if_regs_inst.read_data <= regs[if_regs_inst.addr];
        end
    end

    // Assign internal registers to output (top-level access to registers)
    assign if_top_link.rego = regs;

endmodule

`endif // REGISTER_FILE_SV
