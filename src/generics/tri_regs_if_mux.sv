// ---------------------------------------------------------
// Module: Triple Register Interface Mux
// Description: Mux for the three register interfaces
// Author: André Lamego
// Date: 2025-10-27
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef TRI_REGS_IF_MUX_SV
`define TRI_REGS_IF_MUX_SV

module tri_regs_if_mux(
    input  logic clk,
    input  logic sel,
    regs_if if_out,   // connected to register file (slave)
    regs_if if_in0,   // connected to processor (master)
    regs_if if_in1    // connected to connection monitor (master)
);

    // Master → Slave direction
    always_comb begin
        if_out.write_en   = sel ? if_in1.write_en   : if_in0.write_en;
        if_out.read_en    = sel ? if_in1.read_en    : if_in0.read_en;
        if_out.addr       = sel ? if_in1.addr       : if_in0.addr;
        if_out.write_data = sel ? if_in1.write_data : if_in0.write_data;
    end

    // Slave → Master direction
    always_comb begin
        if_in0.read_data  = if_out.read_data;
        if_in1.read_data  = if_out.read_data;
        if_in0.data_ready = if_out.data_ready;
        if_in1.data_ready = if_out.data_ready;
        if_in0.write_done = if_out.write_done;
        if_in1.write_done = if_out.write_done;
    end

endmodule

`endif // TRI_REGS_IF_MUX_SV
