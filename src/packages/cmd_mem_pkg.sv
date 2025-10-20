// ---------------------------------------------------------
// Module: Command Memory Package
// Description: Defines commands used to configure BLE modules
// as peripherals and enable advertising (HM-10 and HC-05)
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef CMD_MEM_PKG_SV
`define CMD_MEM_PKG_SV

package cmd_mem_pkg;

    // Commands for HM-10 BLE Module (Peripheral Configuration)
    localparam string HM10_CMD_AT        = $sformatf("AT%c\n", 8'h0D);          // Test AT command
    localparam string HM10_CMD_NAME      = $sformatf("AT+NAME%c\n", 8'h0D);     // Set device name
    localparam string HM10_CMD_PIN       = $sformatf("AT+PIN%c\n", 8'h0D);      // Set PIN code
    localparam string HM10_CMD_BAUD      = $sformatf("AT+BAUD%c\n", 8'h0D);     // Set baud rate
    localparam string HM10_CMD_ROLE      = $sformatf("AT+ROLE0%c\n", 8'h0D);    // Set role to peripheral
    localparam string HM10_CMD_RESET     = $sformatf("AT+RESET%c\n", 8'h0D);    // Reset module
    localparam string HM10_CMD_ADV       = $sformatf("AT+ADVEN%c\n", 8'h0D);    // Enable advertising

    // Commands for HC-05 Bluetooth Module (Peripheral Configuration)
    localparam string HC05_CMD_AT        = $sformatf("AT%c\n", 8'h0D);          // Test AT command
    localparam string HC05_CMD_NAME      = $sformatf("AT+NAME=%c\n", 8'h0D);     // Set device name
    localparam string HC05_CMD_PIN       = $sformatf("AT+PSWD=%c\n", 8'h0D);     // Set PIN code
    localparam string HC05_CMD_BAUD      = $sformatf("AT+UART=%c\n", 8'h0D);     // Set baud rate
    localparam string HC05_CMD_ROLE      = $sformatf("AT+ROLE=0%c\n", 8'h0D);   // Set role to slave (peripheral)
    localparam string HC05_CMD_RESET     = $sformatf("AT+RESET%c\n", 8'h0D);    // Reset module

    // Error code
    typedef enum logic [1:0] { 
        NO_ERR      = 2'd0,
        ERR_CMD_WDH = 2'd1,
        ERR_ADDR    = 2'd2,
        ERR_SPACE   = 2'd3
    } cmd_mem_error_t;

    // FSM
    typedef enum logic [4:0] {
        RESET       = 5'd0,  // Load default values of commands to the memory
        IDLE        = 5'd1,  // Waiting for a new byte
        CLEAR       = 5'd2,  // Waiting for a new byte
        GET_ADDR1   = 5'd3,  // Get the first part of the address
        WAIT_ADDR2  = 5'd4,  // Wait for second ASCII address character
        GET_ADDR2   = 5'd5,  // Get the second part of the address
        WAIT_SPACE  = 5'd6,  // Wait for the space between adress and data 
        GET_SPACE   = 5'd7,  // Get the space between adress and data
        WAIT_DATA   = 5'd8,  // Wait for next string to be ready
        GET_DATA    = 5'd9,  // Buffering command string
        WAIT_LF     = 5'd10, // Buffering command string
        CHECK_LF    = 5'd11, // Saw '\r', waiting for '\n'
        STORE       = 5'd12  // Write buffer to memory

    } cmd_mem_state_t;


endpackage // cmd_mem_pkg

`endif // CMD_MEM_PKG_SV
