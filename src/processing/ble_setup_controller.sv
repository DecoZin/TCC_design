// ------------------------------------------------
// Module: BLE Setup Controller
// Description: Finite State Machine for BLE setup
// Author: André Lamego
// Date: 2025-07-03
// ------------------------------------------------
`timescale 1ns / 100ps

`ifndef BLE_SETUP_CONTROLLER_SV
`define BLE_SETUP_CONTROLLER_SV

module ble_setup_controller (
    tmr_if if_ds_timer,                 // Interface instance for wake-up timer
    input  logic [23:0] regs_slp_time_count, // Time to sleep in µs

    //regs_if if_regs_inst,               // TODO: Interface instance

    input  logic clk,                   // Clock signal
    input  logic rst_n,                 // Reset signal

    output logic setting_up,            // Enable BLE setup module
    input  logic setup_done,            // End of setup configuration
    input  logic fail,                  // Error with AT+Commands
    
    input  logic programming,           // Configuring setup commands
    input  logic direct_conn,           // Ignore setup 
    
    input  logic disconnect,            // Disconnection
    input  logic connect,               // Connection established
    input  logic time_out,              // Timeout of Advertisement
    
    input  logic error_pulse,           // Error when programming
    input  logic [1:0] error_code,      // Error of programming
    output logic en_cmd_mem_wr,         // Enable writing in the command memory
    
    output logic [1:0] mux_rx_setup,    // Mux for receiver at setup, 0 for BLE setup, 1 for Command Memory and 2 for Connection Monitor
    output logic mux_tx_setup,          // Mux for trasmitter at setup, 0 for BLE setup, 1 for BLE Setup Controller
    output logic mux_transceiver,       // Mux for transceiver, 0 for setup and 1 for processing
    
    input  logic tx_full,               // Flag of full TX FIFO
    output logic tx_valid,              // Valid byte for sending data to tx
    output logic [7:0] tx_data          // Error message
);
    import ble_ctrl_types_pkg::*;
        
    // Finite State Machine
    ble_ctrl_state_t state = IDLE;

    // Muxes options
    mux_rx_t m_rx_setup;
    mux_tx_t m_tx_setup;
    mux_transceiver_t m_transceiver;

    // Timer Controller
    logic wake_up;
    assign wake_up = if_ds_timer.done;
    assign if_ds_timer.mode = 1'b0;
    assign if_ds_timer.time_count = regs_slp_time_count;

    always_ff @( posedge clk or negedge rst_n ) begin : TIMER
        if ( !rst_n ) begin
            if_ds_timer.enable <= 1'b0;
            if_ds_timer.clear <= 1'b1;
        end else begin
            if (state == IDLE) begin
                if_ds_timer.enable <= 1'b1;
                if_ds_timer.clear <= 1'b0;
            end else begin
                if_ds_timer.enable <= 1'b0;
                if_ds_timer.clear <= 1'b1;
            end
        end
    end

    // Finite State Machine Controller
    always_ff @( posedge clk or negedge rst_n ) begin : FSM
        if ( !rst_n ) begin
            state <= IDLE;
        end else begin
            case ( state )
                IDLE:
                    if (direct_conn) begin
                       state <= CONNECTED; 
                    end else if ( wake_up && !programming ) begin
                        state <= SETUP;
                    end else if ( wake_up && programming ) begin
                        state <= PROGRAMMING;
                    end else begin
                        state <= IDLE;
                    end
                PROGRAMMING:
                    if ( !programming ) begin
                        state <= IDLE;
                    end else if ( error_pulse ) begin
                        state <= ERROR_ACK;
                    end else begin
                        state <= PROGRAMMING;
                    end
                ERROR_ACK:
                    if (tx_valid) begin
                        state <= PROGRAMMING;
                    end else begin
                        state <= ERROR_ACK;
                    end
                SETUP:
                    if ( fail ) begin
                        state <= IDLE;
                    end else if ( setup_done ) begin
                        state <= ADVERTISEMENT;
                    end
                ADVERTISEMENT:
                    if ( connect ) begin
                        state <= CONNECTED;
                    end else if ( !time_out ) begin
                        state <= ADVERTISEMENT;
                    end else begin
                        state <= IDLE;
                    end
                CONNECTED:
                    if ( disconnect && !direct_conn) begin
                        state <= IDLE;
                    end else begin
                        state <= CONNECTED;
                    end
                default: state <= IDLE;
            endcase
        end
    end

    always_comb begin : TC_MUX
        setting_up = 1'b0;
        en_cmd_mem_wr = 1'b0;
        m_transceiver = SETUP_TC;
        m_rx_setup = BLE_SETUP_RX;
        m_tx_setup = BLE_SETUP_TX;
        tx_valid = 1'b0;
        case ( state )
            IDLE: begin
            end
            PROGRAMMING: begin
                m_tx_setup = BLE_CONTROLLER_TX;
                m_rx_setup = CMD_MEM_RX;
                en_cmd_mem_wr = 1'b1;
            end
            ERROR_ACK: begin
                m_tx_setup = BLE_CONTROLLER_TX;
                m_rx_setup = CMD_MEM_RX;
                tx_data = {6'b0, error_code};
                tx_valid = (!tx_full) ? 1'b1 : 1'b0;
            end
            SETUP: begin
                setting_up = 1'b1;
            end
            ADVERTISEMENT: begin
                m_rx_setup = CONN_MON_RX;
            end
            CONNECTED: begin
                m_transceiver = PROCESSOR_TC;
            end
        endcase
    end

    assign mux_rx_setup = {m_rx_setup};
    assign mux_tx_setup = {m_tx_setup};
    assign mux_transceiver = {m_transceiver};

endmodule

`endif // BLE_SETUP_CONTROLLER_SV
