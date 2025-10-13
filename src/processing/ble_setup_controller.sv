// ------------------------------------------------
// Module: BLE Setup Controller
// Description: Finite State Machine for BLE setup
// Author: André Lamego
// Date: 2025-07-03
// ------------------------------------------------
`ifndef BLE_SETUP_CONTROLLER_SV
`define BLE_SETUP_CONTROLLER_SV

module ble_setup_controller (
    input  logic clk,               // Clock signal
    input  logic rst_n,             // Reset signal
    input  logic wake_up,           // Wake up signal
    input  logic programming,       // Configuring setup commands
    input  logic direct_conn,       // Ignore setup 
    input  logic setup_done,        // End of setup configuration
    input  logic fail,              // Error with AT+Commands
    input  logic time_out,          // Timeout of Advertisement
    input  logic connect,           // Connection established
    input  logic disconnect,        // Disconnection
    input  logic error_pulse,       // Error when programming
    input  logic [1:0] error_code,  // Error of programming
    
    output logic en_cmd_mem_wr,     // Enable writing in the command memory
    output logic setting_up,        // Enable BLE setup module
    output logic timer_idle,        // Timer for deep sleep
    output logic timer_adv,         // Timer for advertisement
    output logic [1:0] mux_rx_setup // Mux for receiver at setup, 0 for BLE setup, 1 for Command Memory and 2 for Connection Monitor
    output logic mux_tx_setup       // Mux for trasmitter at setup, 0 for BLE setup, 1 for BLE Setup Controller
    output logic mux_transceiver    // Mux for transceiver, 0 for setup and 1 for processing
    
    input  logic tx_full            // Flag of full TX FIFO
    output logic tx_valid           // Valid byte for sending data to tx
    output logic [7:0] tx_data      // Error message
);

    typedef enum logic [2:0] {
        IDLE            = 3'b000, // Deep sleep
        PROGRAMMING     = 3'b001, // Configuration of setup commands
        ERROR_ACK       = 3'b010, // Configuration of setup commands
        SETUP           = 3'b011, // Setup of name, password, etc
        ADVERTISEMENT   = 3'b100, // Advertisement of BLE
        CONNECTED       = 3'b101  // Connected to BLE Central
    } state_t;

    state_t state = IDLE;

    always_ff @( posedge clk or negedge rst_n ) begin : FSM
        if ( !rst_n ) begin
            timer_idle <= 1'b1;
            timer_adv <= 1'b0;
            en_cmd_mem_wr <= 1'b0;
            mux_transceiver <= 1'b0; // or default to SETUP mode
            state <= IDLE;
        end else begin
            timer_idle <= 1'b0;
            timer_adv <= 1'b0;
            en_cmd_mem_wr <= 1'b0;
            send_ack <= 1'b0;
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
                    if (!tx_full) begin
                        state <= PROGRAMMING;
                    end else begin
                        state <= ERROR_ACK;
                    end
                SETUP:
                    if ( fail ) begin
                        timer_idle <= 1'b1;
                        state <= IDLE;
                    end else if ( setup_done ) begin
                        timer_adv <= 1'b1;
                        state <= ADVERTISEMENT;
                    end
                ADVERTISEMENT:
                    if ( connect ) begin
                        state <= CONNECTED;
                    end else if ( !time_out ) begin
                        state <= ADVERTISEMENT;
                    end else begin
                        timer_idle <= 1'b1;
                        state <= IDLE;
                    end
                CONNECTED:
                    if ( disconnect && !direct_conn) begin
                        timer_idle <= 1'b1;
                        state <= IDLE;
                    end else begin
                        state <= CONNECTED;
                    end
                default:
                    begin
                        timer_idle <= 1'b1;
                        state <= IDLE;
                    end
            endcase
        end
    end

    always_comb begin : TC_MUX
        case ( state )
            IDLE: begin
                mux_transceiver = 1'b0;
                en_cmd_mem_wr = 1'b0;
            end
            PROGRAMMING: begin
                mux_transceiver = 1'b0;
                en_cmd_mem_wr = 1'b1;
            end
            ERROR_ACK: begin
                mux_transceiver = 1'b0;
                en_cmd_mem_wr = 1'b0;
                tx_data = {5'b0, error_code};
                tx_valid = (!tx_full) ? 1'b1 : 1'b0;
            end
            SETUP: begin
                mux_transceiver = 1'b0;
                en_cmd_mem_wr = 1'b0;
            end
            ADVERTISEMENT: begin
                mux_transceiver = 1'b0;
                en_cmd_mem_wr = 1'b0;
            end
            CONNECTED: begin
                mux_transceiver = 1'b1;
                en_cmd_mem_wr = 1'b0;
            end
            default: begin
                mux_transceiver = 1'b0;
            end
        endcase
    end

endmodule

`endif // BLE_SETUP_CONTROLLER_SV
