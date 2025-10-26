// ---------------------------------------------------------
// Module: Processor Types Package
// Description: Defines for the Processor
// Author: André Lamego
// Date: 2025-10-22
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef PROCESSOR_TYPES_SV
`define PROCESSOR_TYPES_SV

package processor_types_pkg;

    // Opcode type definition
    typedef enum logic [7:0] {
        OPC_NOP         = 8'b0000_0000,
        OPC_INTERREGW   = 8'b0000_0010,
        OPC_INTERREGR   = 8'b0001_0001,
        OPC_GLOBALREGW  = 8'b0010_0010,
        OPC_GLOBALREGR  = 8'b0011_0001,
        OPC_JPEGREGW    = 8'b0100_0010,
        OPC_JPEGREGR    = 8'b0101_0001,
        OPC_DISPLAYREGW = 8'b0110_0010,
        OPC_DISPLAYREGR = 8'b0111_0001,
        OPC_SENSREGW    = 8'b1000_0010,
        OPC_SENSREGR    = 8'b1001_0001,
        OPC_GETMEASNUM  = 8'b1010_0000,
        OPC_GETMEAS     = 8'b1011_0001,
        OPC_GETALLMEAS  = 8'b1100_0000,
        OPC_RTMEAS      = 8'b1101_0000,
        OPC_DISPLAYIMG  = 8'b1110_1111
    } opcode_t;

    parameter int MAX_OPDS = 2;

    // Block address offset definitions
    typedef enum logic [1:0] {
        B_GLOBAL_OFFSET  = 2'b00,
        B_JPEG_OFFSET    = 2'b01,
        B_SENS_OFFSET    = 2'b10,
        B_DISPLAY_OFFSET = 2'b11
    } addr_offset_t;

    // Specific address definitions
    typedef enum logic [7:0] {
        A_NMEAS      = 8'b0000_0001,
        A_GETMEAS    = 8'b0000_0010,
        A_ALLMEAS    = 8'b0000_0011
    } bus_addr_t;
    
    parameter logic [7:0] ALLMEAS_SIZE = 8'b0000_0011;

    // State definitions
    typedef enum logic [2:0] {
        S_DISCONNECTED,
        S_IDLE,
        S_DECODE_CMD,
        S_GET_OPDS,
        S_MEM,
        S_DELAY,
        S_ACK
    } processor_state_t;

    // Use both interfaces; selecting between them by 'mem_target' flag.
    typedef enum logic [1:0] { TARGET_NONE, TARGET_SPECIAL, TARGET_BUS } mem_target_t;

endpackage // processor_types_pkg
`endif
