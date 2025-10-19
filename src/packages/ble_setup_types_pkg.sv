// ---------------------------------------------------------
// Module: Command Memory Package
// Description: Defines for commands and registers used to
// setup the BLE module
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef BLE_SETUP_TYPES_PKG_SV
`define BLE_SETUP_TYPES_PKG_SV
    
package ble_setup_types_pkg;

    // Finite State Machine
    typedef enum logic [3:0] { 
        IDLE            = 'd1, // Not in setup state
        GET_CMD_NUMBER  = 'd2, // Get the number of commands in memory
        SEND_CMD        = 'd3, // Send the command byte/byte to TX FIFO until \r\n
        WAIT_ACK        = 'd4, // Wait for the acknowledge byte
        GET_ACK         = 'd5, // Get the acknowledge byte and put it in the buffer
        EVALUATE_ACK    = 'd6, // Evaluate the acknowledge has OK in the first 5 bytes or has a ERROR
        FLUSH_RX        = 'd7  // Flush the RX FIFO after knowing the result if the ack is longer than 5 bytes "OK+..."
    } ble_setup_state_t;

    // Muxes Options
    typedef enum logic [1:0] {
        BLE_SETUP_RX,
        CMD_MEM_RX,
        CONN_MON_RX,
        UNSUPPORTED_RX
    } mux_rx_t;

    typedef enum logic {
        BLE_SETUP_TX,
        BLE_CONTROLLER_TX
    } mux_tx_t;

    typedef enum logic {
        SETUP_TC,
        PROCESSOR_TC
    } mux_transceiver_t;


endpackage // ble_setup_types_pkg

`endif // BLE_SETUP_TYPES_PKG_SV
