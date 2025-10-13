// ---------------------------------------------------
// Module: Communication
// Description: Finite State Machine for communication 
// Author: André Lamego
// Date: 2025-03-07
// ---------------------------------------------------
`ifndef COMMUNICATION_SV
`define COMMUNICATION_SV

module communication (
    input  logic        clk,               // Clock signal
    input  logic        rst_n,             // Reset signal
    input  logic        start_bit,         // Start bit from UART
    input  logic        valid_cmd,         // Valid command received
    input  logic        time_out,          // Timeout of timer
    input  logic        last_opds,         // Last operand received
    input  logic        connect,           // Connection established
    input  logic        disconnect,        // Disconnection
    output logic        timer_idle,        // Timer for time out disconnection
    output logic        timer_gopds,       // Timer for getting the operands
    output logic        timer_delay,       // Timer for getting the operands
    output logic        opds_counter_rst,  // Reset operands counter
    output logic        send_ack,          // Send acknowledge to transceiver
    output logic [2:0]  mux_timer          // Timer for getting the operands
);

    typedef enum logic [2:0] {
        IDLE        = 3'b000, // Waiting for data
        DECODE_CMD  = 3'b001, // Decoding first data as a command
        GET_OPDS    = 3'b010, // Parsing next data as operands
        MEM         = 3'b011,  // Enable memory acess
        DELAY       = 3'b100,  // Wait for the longest memory acess period
        ACK         = 3'b101,  // Send acknowledge to transceiver
        DISCONNECT  = 3'b110  // Waits for reconnection
    } state_t;

    state_t state = DISCONNECT;

    always_ff @( posedge clk or negedge rst_n ) begin : FSM
        if ( !rst_n ) begin
            state <= DISCONNECT;
        end else begin
            timer_idle = 1'b0;
            timer_gopds = 1'b0;
            timer_delay = 1'b0;
            opds_counter_rst = 1'b0;
            send_ack = 1'b0;
            case ( state )
                IDLE:
                if (disconnect) begin
                    state = DISCONNECT;
                end else if ( start_bit ) begin
                    state = DECODE_CMD;
                end else begin
                    timer_idle = 1'b1;
                    state = IDLE;
                end
                DECODE_CMD:
                if (valid_cmd) begin
                    timer_gopds = 1'b1;
                    state = GET_OPDS;
                end else begin
                    timer_idle = 1'b1;
                    state = IDLE;
                end
                GET_OPDS:
                if (time_out) begin
                    timer_idle = 1'b1;
                    state = IDLE;
                end else if (last_opds) begin
                    state = MEM;
                end else begin
                    state = GET_OPDS;
                end
                MEM:
                begin
                    timer_delay = 1'b1;
                    state = DELAY;
                end
                DELAY:
                if (time_out) begin
                    state = ACK;
                end else begin
                    state = DELAY;
                end
                ACK:
                    state = IDLE;           
                DISCONNECT:
                if (connect) begin
                    timer_idle = 1'b1;
                    state = IDLE;
                end else begin
                    state = DISCONNECT;
                end
                default:
                    state = DISCONNECT;
            endcase
        end
    end

    always_comb begin : TC_MUX
        case ( state )
            IDLE:
                mux_timer = 2'b00;
            DECODE_CMD:
                mux_timer = 2'b00;
            GET_OPDS:
                mux_timer = 2'b01;
            MEM:
                mux_timer = 2'b00;
            DELAY:
                mux_timer = 2'b10;
            ACK:
                mux_timer = 2'b00;
            DISCONNECT:
                mux_timer = 2'b00;
        default:
                mux_timer = 2'b00;
        endcase
    end

endmodule

`endif // COMMUNICATION_SV
