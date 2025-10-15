// ---------------------------------------------------------
// Module: FIFO
// Description: First In First Out (FIFO) buffer module
// Author: André Lamego
// Date: 2025-04-08
// ---------------------------------------------------------
`ifndef FIFO_SV
`define FIFO_SV

module fifo #(
    parameter WIDTH = 8,  // Data width
    parameter DEPTH = 64  // FIFO depth
) (
    input  logic             clk,
    input  logic             rst_n,
    input  logic             i_wr_en,      // Write enable
    input  logic [WIDTH-1:0] i_wr_data,    // Data to write
    input  logic             i_rd_en,      // Read enable
    
    output logic o_ready_pulse,
    output logic [WIDTH-1:0] o_rd_data,    // Data to read
    output logic             o_full,       // FIFO full flag
    output logic             o_empty       // FIFO empty flag
);

    // Internal signals
    logic [WIDTH-1:0] mem [0:DEPTH-1]; // FIFO memory
    logic [$clog2(DEPTH)-1:0] wr_ptr;    // Write pointer
    logic [$clog2(DEPTH)-1:0] rd_ptr;    // Read pointer
    logic [$clog2(DEPTH)-1:0] count;     // Number of elements in FIFO
    logic full, empty;

    // Write logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 0;
        end else if (i_wr_en && !full) begin
            mem[wr_ptr] <= i_wr_data;
            wr_ptr <= ({ ($bits(DEPTH)-$bits(wr_ptr ))'('0),wr_ptr } == DEPTH-1) ? 0 : wr_ptr + 1; // Wrap-around logic
        end
    end

    // Read logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= 0;
            o_rd_data <= '0;
            o_ready_pulse <= 0;
        end else if (i_rd_en && !empty) begin
            o_rd_data <= mem[rd_ptr];
            rd_ptr <= ({ ($bits(DEPTH)-$bits(rd_ptr ))'('0),rd_ptr } == DEPTH-1) ? 0 : rd_ptr + 1;
            o_ready_pulse <= 1;  // Pulse high for 1 cycle when reading valid data
        end else begin
            o_ready_pulse <= 0;
        end
    end

    // Count logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count <= 0;
        end else if (i_wr_en && !full && i_rd_en && !empty) begin
            count <= count; // Simultaneous read and write
        end else if (i_wr_en && !full) begin
            count <= count + 1;
        end else if (i_rd_en && !empty) begin
            count <= count - 1;
        end
    end

    // Status flags
    assign full  = ({ ($bits(DEPTH)-$bits(count ))'('0),count } == DEPTH);
    assign empty = (count == 0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_full  <= 0;
            o_empty <= 1;
        end else begin
            o_full  <= full;
            o_empty <= empty;
        end
    end

endmodule

`endif // FIFO_SV
