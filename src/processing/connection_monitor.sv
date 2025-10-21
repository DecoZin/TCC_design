// ----------------------------------------------------------------------------
// Module: BLE Connection Monitor (revised)
// Description: Monitors the connection state, detects OK+CONN and OK+DISC,
//              parses MAC ":XXXXXXXXXXXX\r\n" and writes MAC to special regs.
// Author: Revised by ChatGPT (based on André Lamego draft)
// Date: 2025-10-07
// ----------------------------------------------------------------------------
`timescale 1ns / 100ps

`ifndef BLE_CONNECTION_MONITOR
`define BLE_CONNECTION_MONITOR

module connection_monitor (
    // Interfaces
    regs_if   if_regs_inst, // interface to Special Regs
    tmr_if    if_tmr,       // interface to timer

    // Clock and Reset
    input  logic clk,   // Clock signal
    input  logic rst_n, // Reset signal, active low

    // UART RX (handshake: valid indicates data present; ack_ready indicates data sampled/consumed)
    input  logic [7:0] ack_byte,  // byte from UART RX
    input  logic       ack_valid, // pulse when byte is available to read
    input  logic       ack_ready, // signal from RX indicating data has been accepted
    output logic       get_ack_byte, // request/rd_en to sample the next RX byte

    // Control signals
    input  logic setup_done,    // asserted by BLE setup when advertisement begins
    output logic connect,       // asserted when connected
    output logic disconnect,    // pulse/level when disconnected
    output logic timeout,       // pulse/level if advertisement timed out

    // Timer parameters
    input  logic [23:0] reg_adv_time_count,  // advertisement timeout
    input  logic [23:0] reg_conn_time_count  // connected inactivity timeout
);
    import conn_monitor_types_pkg::*;
    import ei_mem_pkg::*;

    // ------------------------------------------------------------------------
    // Local declarations
    // ------------------------------------------------------------------------

    conn_monitor_state_t state, next_state;

    // Rolling buffers (monitor small window and mac accumulation)
    logic [7:0] rx_monitor_buffer [0:6]; // last 7 chars for pattern OK+CONN / OK+DISC
    logic [7:0] rx_mac_buffer     [0:14]; // rolling 15-char buffer for ":XXXXXXXXXXXX\r\n"

    // flags derived from buffers
    logic ok_conn;
    logic ok_disc;
    logic ok_mac;

    // MAC assembly
    logic [7:0] ascii_mac [0:11];    // 12 ASCII hex chars
    assign ascii_mac = rx_mac_buffer[1:12];

    logic [7:0] mac_byte  [0:5];     // final 6 bytes
    logic [3:0] nibble_hi, nibble_lo;
    logic mac_valid;

    // writing MAC to regs
    logic [2:0] mac_write_idx;
    logic       mac_write_in_progress;
    logic       mac_written;

    // control signals
    logic clear_tmr;
    logic use_conn_timeout;

    // internal handshake
    logic reading_byte;

    // ------------------------------------------------------------------------
    // Reset / sequential FSM
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
        end else begin
            state <= next_state;
        end
    end

    // ------------------------------------------------------------------------
    // Next-state logic
    // ------------------------------------------------------------------------
    always_comb begin
        // default
        next_state = state;

        case (state)
            S_IDLE: begin
                if (setup_done) next_state = S_WAIT_EVENT;
            end

            S_WAIT_EVENT: begin
                if (if_tmr.done) next_state = S_IDLE;      // advertisement timeout -> back to idle
                else if (ok_conn) next_state = S_PARSE_MAC; // found OK+CONN -> parse MAC
            end

            S_PARSE_MAC: begin
                if (if_tmr.done) next_state = S_IDLE;      // timeout while parsing -> idle
                else if (ok_mac) next_state = S_STORE_MAC; // mac parsed -> store
            end

            S_STORE_MAC: begin
                if (mac_written) next_state = S_CONNECTED;
            end

            S_CONNECTED: begin
                if (ok_disc) next_state = S_HANDLE_DISCONNECT;
                else if (if_tmr.done) next_state = S_HANDLE_DISCONNECT; // inactivity timeout -> disconnect
            end

            S_HANDLE_DISCONNECT: begin
                next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    // ------------------------------------------------------------------------
    // FSM outputs and timer control
    // ------------------------------------------------------------------------
    assign if_tmr.clear = clear_tmr || state != next_state || ack_ready;

    always_comb begin
        // defaults
        clear_tmr = 1'b0;
        if_tmr.enable = 1'b0;
        if_tmr.time_count = reg_adv_time_count;
        if_tmr.mode = 1'b0; // one-shot by default
        connect = 1'b0;
        mac_write_in_progress = 1'b0;

        case (state)
            S_IDLE: begin
                // timer stopped, clean buffers
                if_tmr.enable = 1'b0;
                clear_tmr = 1'b1;
            end

            S_WAIT_EVENT: begin
                // start advertisement timer
                if_tmr.time_count = reg_adv_time_count;
                if_tmr.enable = 1'b1;
            end

            S_PARSE_MAC: begin
                // keep advertisement timer running while parsing
                if_tmr.time_count = reg_adv_time_count;
                if_tmr.enable = 1'b1;
                // request bytes when available
            end

            S_STORE_MAC: begin
                if_tmr.enable = 1'b0;
                // handle in sequential block
            end

            S_CONNECTED: begin
                // enable connected inactivity timer
                if_tmr.time_count = reg_conn_time_count;
                if_tmr.enable = 1'b1;
                connect = 1'b1;
            end

            S_HANDLE_DISCONNECT: begin
                // assert a disconnect pulse in sequential block
                clear_tmr = 1'b1;
            end
        endcase
    end

    // ------------------------------------------------------------------------
    // Rolling buffers: capture incoming bytes
    // - when ack_valid pulses, request to latch by asserting get_ack_byte for 1 cycle
    // - when ack_ready (or next cycle), shift buffers and append ack_byte
    // ------------------------------------------------------------------------
    // reading_byte used to assert get_ack_byte for handshake
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || state == S_IDLE) begin
            integer i;
            get_ack_byte <= 1'b0;
            for (i=0; i<7; i=i+1) rx_monitor_buffer[i] <= 8'd0;
            for (i=0; i<15; i=i+1) rx_mac_buffer[i] <= 8'd0;
            reading_byte <= 1'b0;
        end else begin
            // When a new byte arrives (ack_valid), we assert get_ack_byte for one cycle
            if (state != S_CONNECTED) begin
                if (ack_valid && !reading_byte) begin
                    get_ack_byte <= 1'b1;
                    reading_byte <= 1'b1;
                end else begin
                    get_ack_byte <= 1'b0;
                end
            end else begin
                reading_byte <= 1'b1;
            end

            // Accept the byte when ack_ready (or on the same cycle if ack_ready coexists)
            if (reading_byte && ack_ready) begin
                // shift rx_mac_buffer left and append new byte
                integer j;
                for (j=0; j<14; j=j+1) rx_mac_buffer[j] <= rx_mac_buffer[j+1];
                rx_mac_buffer[14] <= ack_byte;

                // shift rx_monitor_buffer
                for (j=0; j<6; j=j+1) rx_monitor_buffer[j] <= rx_monitor_buffer[j+1];
                rx_monitor_buffer[6] <= ack_byte;

                reading_byte <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Pattern detectors (combinational)
    // ------------------------------------------------------------------------
    function automatic logic is_conn_found(input logic [7:0] w [0:6]);
        return (w[0] == "O" && w[1] == "K" && w[2] == "+" &&
                w[3] == "C" && w[4] == "O" && w[5] == "N" && w[6] == "N");
    endfunction

    function automatic logic is_disc_found(input logic [7:0] w [0:6]);
        return (w[0] == "O" && w[1] == "K" && w[2] == "+" &&
                w[3] == "D" && w[4] == "I" && w[5] == "S" && w[6] == "C");
    endfunction

    // check if rx_mac_buffer holds pattern : + 12 hex chars + CR LF
    function automatic logic is_mac_found(input logic [7:0] w [0:14]);
        // positions: w[0] .. w[14] ; expect w[0]==':', w[13]==0x0D (CR), w[14]==0x0A (LF)
        logic ok;
        ok = (w[0] == ":" && w[13] == 8'h0D && w[14] == 8'h0A);
        return ok;
    endfunction

    assign ok_conn = is_conn_found(rx_monitor_buffer);
    assign ok_disc = is_disc_found(rx_monitor_buffer);
    assign ok_mac  = is_mac_found(rx_mac_buffer);

    // ------------------------------------------------------------------------
    // ASCII hex -> nibble function
    // ------------------------------------------------------------------------
    function automatic logic [3:0] ascii_hex_to_nibble(input logic [7:0] c);
        if ((c >= "0") && (c <= "9")) ascii_hex_to_nibble = 4'(c - "0");
        else if ((c >= "A") && (c <= "F")) ascii_hex_to_nibble = 4'(c - "A") + 4'hA;
        else if ((c >= "a") && (c <= "f")) ascii_hex_to_nibble = 4'(c - "a") + 4'hA;
        else ascii_hex_to_nibble = 4'hF; // invalid marker
        return ascii_hex_to_nibble;
    endfunction

    // ------------------------------------------------------------------------
    // PARSE MAC: when ok_mac asserted, extract 12 ASCII hex characters from rx_mac_buffer[1..12]
    // and convert into 6 bytes mac_byte[0..5] by pairing nibbles
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            integer k;
            for (k=0; k<6; k=k+1) mac_byte[k] <= 8'd0;
            mac_valid <= 1'b0;
        end else begin
            if (state == S_PARSE_MAC && ok_mac) begin
                // convert ascii chars rx_mac_buffer[1..12] into 6 bytes
                integer idx;
                logic [3:0] hi, lo;
                mac_valid <= 1'b1;
                for (idx = 0; idx < 6; idx = idx + 1) begin
                    hi = ascii_hex_to_nibble(ascii_mac[1 + 2*idx]);
                    lo = ascii_hex_to_nibble(ascii_mac[2*idx]);
                    // if invalid nibble, mark mac_valid false
                    if ((hi == 4'hF) || (lo == 4'hF)) begin
                        mac_byte[idx] <= 8'h00;
                        mac_valid <= 1'b0;
                    end else begin
                        mac_byte[idx] <= {lo, hi}; // combine into 8-bit byte
                    end
                end
            end else begin
                mac_valid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // STORE MAC into special registers (simple single-cycle writes, one per state transition)
    // This assumes if_regs_inst.write_en, write_data, addr are available and synchronous.
    // We step through mac indices and assert write_en for one cycle each.
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mac_write_idx <= 3'd0;
            mac_written <= 1'b0;
            if_regs_inst.write_en <= 1'b0;
            if_regs_inst.addr <= EIR_CENTRAL_MAC0;
            if_regs_inst.write_data <= 8'd0;
        end else begin
            if (state == S_STORE_MAC) begin
                if (!mac_written) begin
                    // start write sequence
                    if_regs_inst.write_en <= 1'b1;
                    case (mac_write_idx)
                        3'd0: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC0; 
                            if_regs_inst.write_data <= mac_byte[0]; 
                        end
                        3'd1: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC1; 
                            if_regs_inst.write_data <= mac_byte[1]; 
                        end
                        3'd2: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC2; 
                            if_regs_inst.write_data <= mac_byte[2]; 
                        end
                        3'd3: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC3; 
                            if_regs_inst.write_data <= mac_byte[3]; 
                        end
                        3'd4: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC4; 
                            if_regs_inst.write_data <= mac_byte[4]; 
                        end
                        3'd5: begin 
                            if_regs_inst.addr <= EIR_CENTRAL_MAC5; 
                            if_regs_inst.write_data <= mac_byte[5]; 
                        end
                        default: begin end
                    endcase

                    if (mac_write_idx > 3'd5) begin
                        mac_write_idx <= 3'd0;
                        mac_written <= 1'b1;
                        if_regs_inst.write_en <= 1'b0;
                    end else begin
                        mac_write_idx <= mac_write_idx + 1;
                    end
                end
            end else begin
                // idle reset
                mac_write_idx <= 3'd0;
                mac_written <= 1'b0;
                if_regs_inst.write_en <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // CONNECT / DISCONNECT / TIMEOUT pulses (registered)
    // ------------------------------------------------------------------------
    // connect asserted in CONNECTED state; disconnect/timeouts pulsed in transitions

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            disconnect <= 1'b0;
            timeout <= 1'b0;
        end else begin
            // clear pulses by default
            disconnect <= 1'b0;
            timeout <= 1'b0;

            // generate on entry to S_HANDLE_DISCONNECT / when if_tmr.done in WAIT_EVENT
            if (state == S_HANDLE_DISCONNECT) begin
                disconnect <= 1'b1;
            end

            if (state == S_WAIT_EVENT && if_tmr.done) begin
                // advertisement timeout
                timeout <= 1'b1;
            end
        end
    end


endmodule

`endif // BLE_CONNECTION_MONITOR
