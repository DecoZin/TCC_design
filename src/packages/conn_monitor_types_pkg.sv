// ---------------------------------------------------------
// Module: Connection Monitor Package
// Description: Types used in the Connection Monitor
// Author: André Lamego
// Date: 2025-10-20
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef CONN_MONITOR_TYPES_PKG_SV
`define CONN_MONITOR_TYPES_PKG_SV
    
package conn_monitor_types_pkg;

    // Finite State Machine
    typedef enum logic [2:0] {
        S_IDLE,
        S_WAIT_EVENT,
        S_PARSE_MAC,
        S_STORE_MAC,
        S_CONNECTED,
        S_HANDLE_DISCONNECT
    } conn_monitor_state_t;


endpackage // conn_monitor_types_pkg

`endif // CONN_MONITOR_TYPES_PKG_SV
