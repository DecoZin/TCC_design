// ---------------------------------------------------------
// Module: Command Memory Package
// Description: Defines for commands and registers used to
// setup the BLE module
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef BLE_CTRL_TYPES_PKG_SV
`define BLE_CTRL_TYPES_PKG_SV
    
package ble_ctrl_types_pkg;

    // Finite State Machine
    typedef enum logic [2:0] {
        IDLE            = 3'b000, // Deep sleep
        PROGRAMMING     = 3'b001, // Configuration of setup commands
        ERROR_ACK       = 3'b010, // Configuration of setup commands
        SETUP           = 3'b011, // Setup of name, password, etc
        ADVERTISEMENT   = 3'b100, // Advertisement of BLE
        CONNECTED       = 3'b101  // Connected to BLE Central
    } ble_ctrl_state_t;

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


endpackage // ble_ctrl_types_pkg

`endif // BLE_CTRL_TYPES_PKG_SV
