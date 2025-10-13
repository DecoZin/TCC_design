// ----------------------------------------------------------------------------
// Module: Processador Interface Externa
// Description: Interpreta comandos vindos da interface externa (UART),
//              acessa registradores via regs_if, encaminha dados de imagem
//              para o bloco Córnea Opaca e envia acknowledgements via TX.
// Author: André Lamego
// Date: 2025-10-08
// ----------------------------------------------------------------------------
`timescale 1ns / 1ps

module proc_if_ext #(
    parameter ADDR_WIDTH = 8,
    parameter DATA_WIDTH = 8
)(
    // clock / reset
    input logic clk,
    input logic rst_n,

    // Interfaces regs (master modport)
    regs_if if_special_regs, // special regs (registers of interface)
    regs_if if_bus,          // bus regs interface (to other blocks)

    // timer interface (controller modport)
    tmr_if if_timer,

    // connection signals (from connection monitor)
    input  logic connect,
    input  logic disconnect,

    // UART RX (from external interface)
    input  logic       data_valid,  // idicates that a data is available
    input  logic       data_ready,  // pulse when a byte is ready to be read
    input  logic [7:0] data_byte,   // received byte
    output logic       get_data,    // request to read / rd_en

    // UART TX (to external interface)
    input logic        tx_full,     // tx can't handle more data
    output logic [7:0] ack_byte,    // byte to send
    output logic       ack_valid,   // pulse to indicate ack_byte valid

    // Image path (to cornea_opaca)
    output logic [7:0] image_data,
    output logic       img_data_valid,

    // timing config (from regs)
    input  logic [23:0] regs_opds_time_count,
    input  logic [23:0] regs_delay_time_count
);

    // ------------------------------------------------------------------------
    // Opcode definitions
    // ------------------------------------------------------------------------
    localparam logic [7:0] OPC_INTERREGW   = 8'b0000_0010;
    localparam logic [7:0] OPC_INTERREGR   = 8'b0001_0001;
    localparam logic [7:0] OPC_GLOBALREGW  = 8'b0010_0010;
    localparam logic [7:0] OPC_GLOBALREGR  = 8'b0011_0001;
    localparam logic [7:0] OPC_JPEGREGW    = 8'b0100_0010;
    localparam logic [7:0] OPC_JPEGREGR    = 8'b0101_0001;
    localparam logic [7:0] OPC_DISPLAYREGW = 8'b0110_0010;
    localparam logic [7:0] OPC_DISPLAYREGR = 8'b0111_0001;
    localparam logic [7:0] OPC_SENSREGW    = 8'b1000_0010;
    localparam logic [7:0] OPC_SENSREGR    = 8'b1001_0001;
    localparam logic [7:0] OPC_GETMEASNUM  = 8'b1010_0000;
    localparam logic [7:0] OPC_GETMEAS     = 8'b1011_0001;
    localparam logic [7:0] OPC_GETALLMEAS  = 8'b1100_0000;
    localparam logic [7:0] OPC_RTMEAS      = 8'b1101_0000;
    localparam logic [7:0] OPC_DISPLAYIMG  = 8'b1110_1111;

    // ------------------------------------------------------------------------
    // Block address offset definitions
    // ------------------------------------------------------------------------
    localparam logic [1:0] B_GLOBAL_OFFSET  = 2'b00;
    localparam logic [1:0] B_JPEG_OFFSET    = 2'b01;
    localparam logic [1:0] B_SENS_OFFSET    = 2'b10;
    localparam logic [1:0] B_DISPLAY_OFFSET = 2'b11;
    
    // ------------------------------------------------------------------------
    // Specific address definitions
    // ------------------------------------------------------------------------
    localparam logic [7:0] A_NMEAS = `A_NMEAS;
    localparam logic [7:0] A_ALLMEAS = `A_ALLMEAS;
    localparam logic [7:0] A_GETMEAS = `A_GETMEAS;
    localparam logic [7:0] ALLMEAS_SIZE = `ALLMEAS_SIZE;

    // ------------------------------------------------------------------------
    // FSM States
    // ------------------------------------------------------------------------
    typedef enum logic [2:0] {
        S_DISCONNECTED,
        S_IDLE,
        S_DECODE_CMD,
        S_GET_OPDS,
        S_MEM,
        S_DELAY,
        S_ACK
    } state_t;

    state_t state, next_state;

    // ------------------------------------------------------------------------
    // Internal buffers & counters
    // ------------------------------------------------------------------------
    localparam int MAX_OPDS = 2;

    logic [7:0] cmd_opcode;
    logic [7:0] opds_buf [0:MAX_OPDS-1];
    integer     opds_expected;    // how many operand bytes expected for this opcode
    integer     opds_received;    // current count
    integer     image_bytes_left; // for DISPLAYIMG
    logic       cmd_valid;

    // memory/reg access temp signals
    logic [7:0] mem_addr;
    logic [7:0] mem_wr_data;
    logic       mem_read_pending;
    logic [7:0] mem_read_data; // from regs IF (if_special_regs.read_data or if_bus.read_data)

    // ack response buffer (we build bytes to send via ack_byte / ack_valid)
    logic [7:0] ack_buf [0:127];
    integer     ack_len;
    integer     ack_idx;

    // control flags
    logic   collecting_cmd;  // when expecting opcode/operands
    logic   collecting_opds;  // when expecting opcode/operands
    logic   opds_timeout;    // operand timeout flag
    logic   delay_timeout;   // delay timeout flag
    integer opds_timeout_cnt;
    logic   ack_sent;
    logic   last_opds;
    logic   ack_ready;
    logic   special_read_done;
    logic   bus_read_done;
    logic   special_write_done;
    logic   bus_write_done;
    logic   got_all_meas;
    integer all_meas_idx;

    // Use both interfaces; selecting between them by 'mem_target' flag.
    typedef enum logic [1:0] { TARGET_NONE, TARGET_SPECIAL, TARGET_BUS } mem_target_t;
    mem_target_t mem_target;

    // ------------------------------------------------------------------------
    // Current state logic
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_DISCONNECTED;
        end else begin
            state <= next_state;
        end
    end

    // ------------------------------------------------------------------------
    // Determine how many operands expected for each opcode
    // ------------------------------------------------------------------------
    function automatic int opcode_opds_count(input logic [7:0] op);
        case (op)
            OPC_INTERREGW, OPC_GLOBALREGW, OPC_JPEGREGW,
            OPC_DISPLAYREGW, OPC_SENSREGW: opcode_opds_count = 2; // DATA, ADDR
            OPC_INTERREGR, OPC_GLOBALREGR, OPC_JPEGREGR,
            OPC_DISPLAYREGR, OPC_SENSREGR: opcode_opds_count = 1; // ADDR
            OPC_GETMEASNUM, OPC_GETALLMEAS: opcode_opds_count = 0;
            OPC_GETMEAS, OPC_RTMEAS: opcode_opds_count = 1; // ADDR
            OPC_DISPLAYIMG: opcode_opds_count = 1; // first op indicates N (image length) but actually dynamic
            default: opcode_opds_count = -1; // invalid/unknown
        endcase
        return opcode_opds_count;
    endfunction

    // ------------------------------------------------------------------------
    // Next-state logic
    // ------------------------------------------------------------------------
    always_comb begin
        // defaults
        next_state = state;
        last_opds = (opds_received >= opds_expected) && 
        ((opc_is_displayimg() && image_bytes_left == 0) || !opc_is_displayimg());
        ack_ready = special_read_done || bus_read_done || 
                    special_write_done || bus_write_done ||
                    opc_is_displayimg();
        special_read_done  = mem_read_pending  && 
                             (mem_target == TARGET_SPECIAL) && 
                             if_special_regs.data_ready;
        bus_read_done      = mem_read_pending  &&
                             (mem_target == TARGET_BUS) &&
                             if_bus.data_ready;
        special_write_done = !mem_read_pending &&
                             (mem_target == TARGET_SPECIAL) &&
                             if_special_regs.write_done;
        bus_write_done     = !mem_read_pending &&
                             (mem_target == TARGET_BUS) &&
                             if_bus.write_done;
        case (state)
            S_DISCONNECTED: begin
                if (connect) next_state = S_IDLE;
            end

            S_IDLE: begin
                if (disconnect) next_state = S_DISCONNECTED;
                else if (data_valid) next_state = S_DECODE_CMD;
            end

            S_DECODE_CMD: begin
                if (cmd_valid) begin
                    if (opds_expected == 0) next_state = S_MEM;
                    else next_state = S_GET_OPDS;
                end else next_state = S_IDLE;
            end

            S_GET_OPDS: begin
                if (opds_timeout) next_state = S_IDLE;
                else if (last_opds) next_state = S_MEM;
            end

            S_MEM: begin
                next_state = S_DELAY;
            end

            S_DELAY: begin
                // wait for delay timeout or mem_read completion
                if (delay_timeout) next_state = S_ACK;
                else if (ack_ready) next_state = S_ACK;
            end

            S_ACK: begin
                if (ack_sent) next_state = S_IDLE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    function automatic logic opc_is_displayimg();
        return (cmd_opcode == OPC_DISPLAYIMG);
    endfunction

    // ------------------------------------------------------------------------
    // Command and operand collection logic
    // - On S_DECODE_CMD: capture opcode from data_byte (single byte)
    // - On S_GET_OPDS: collect operands, support dynamic length for DISPLAYIMG
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_opcode <= 8'd0;
            cmd_valid <= 1'b0;
            opds_expected <= 0;
            opds_received <= 0;
            image_bytes_left <= 0;
            collecting_cmd <= 1'b0;
            collecting_opds <= 1'b0;
            img_data_valid <= 1'b0;
            get_data <= 1'b0;
        end else begin
            if (state == S_DECODE_CMD) begin
                // capture opcode from data_byte (one-cycle handshake)
                get_data <= 1'b0;
                collecting_opds <= 1'b0;
                if (data_valid && !collecting_cmd) begin
                    // request the byte (acknowledge to RX)
                    get_data <= 1'b1; // single-cycle request; actual latching assumed by upstream
                    collecting_cmd <= 1'b1;
                end else if (collecting_cmd && data_ready) begin
                    collecting_cmd <= 1'b0;
                    cmd_opcode <= data_byte;
                    opds_expected <= opcode_opds_count(data_byte);
                    cmd_valid <= (opds_expected == -1) ? 1'b0 : 1'b1;
                    opds_received <= 0;
                    image_bytes_left <= 0;
                end
            end else if (state == S_GET_OPDS) begin
                get_data <= 1'b0;
                img_data_valid <= 1'b0;
                if (data_valid && !collecting_opds) begin
                    get_data <= 1'b1;
                    collecting_opds <= 1'b1;
                end else if (collecting_opds && data_ready) begin
                    collecting_opds <= 1'b0;
                    opds_received <= opds_received + 1;
                    if (opc_is_displayimg() && opds_received == 0) begin
                        image_bytes_left <= data_byte;
                        opds_expected <= 1 + data_byte;
                    end else if (opc_is_displayimg()) begin
                        image_bytes_left <= image_bytes_left - 1;
                        image_data <= data_byte;
                        img_data_valid <= 1'b1;
                    end else opds_buf[opds_received] <= data_byte;
                end
            end else begin
                    cmd_valid <= 1'b0;
                    collecting_cmd <= 1'b0;
                    collecting_opds <= 1'b0;
                    get_data <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // MEM state actions: perform access
    // This block performs the requested operation based on cmd_opcode and opds_buf
    // ------------------------------------------------------------------------
    assign if_special_regs.write_data = opds_buf[1];
    assign if_special_regs.addr = opds_buf[0];
    assign if_bus.write_data = opds_buf[1];
    
    always_comb begin
        case (cmd_opcode)
            OPC_GLOBALREGW:  if_bus.addr = {B_GLOBAL_OFFSET , opds_buf[0]};
            OPC_GLOBALREGR:  if_bus.addr = {B_GLOBAL_OFFSET , opds_buf[0]};
            OPC_JPEGREGW:    if_bus.addr = {B_JPEG_OFFSET   , opds_buf[0]};
            OPC_JPEGREGR:    if_bus.addr = {B_JPEG_OFFSET   , opds_buf[0]};
            OPC_DISPLAYREGW: if_bus.addr = {B_DISPLAY_OFFSET, opds_buf[0]};
            OPC_DISPLAYREGR: if_bus.addr = {B_DISPLAY_OFFSET, opds_buf[0]};
            OPC_SENSREGW:    if_bus.addr = {B_SENS_OFFSET   , opds_buf[0]};
            OPC_SENSREGR:    if_bus.addr = {B_SENS_OFFSET   , opds_buf[0]};
            OPC_GETMEASNUM:  if_bus.addr = {B_SENS_OFFSET   , A_NMEAS};
            OPC_GETMEAS:     if_bus.addr = {B_SENS_OFFSET   , opds_buf[0]};
            OPC_GETALLMEAS:  if_bus.addr = {B_SENS_OFFSET   , A_ALLMEAS};
            OPC_RTMEAS:      if_bus.addr = {B_SENS_OFFSET   , A_GETMEAS};
            default: 
        endcase
    end
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_target <= TARGET_NONE;
            mem_read_pending <= 1'b0;
            for (i=0; i<2; i=i+1) opds_buf[i] <= 8'd0;
            if_special_regs.write_en   <= 1'b0;
            if_special_regs.read_en    <= 1'b0;
            if_bus.write_en   <= 1'b0;
            if_bus.read_en    <= 1'b0;
        end else begin
            // Reset flags
            if (state == S_IDLE) mem_target <= TARGET_NONE;
            if (state == S_IDLE) mem_read_pending <= 1'b0;
            
            // Reset special registers
            if_special_regs.write_en   <= 1'b0;
            if_special_regs.read_en    <= 1'b0;
            
            // Reset bus registers
            if_bus.write_en   <= 1'b0;
            if_bus.read_en    <= 1'b0;

            if (state == S_MEM) begin
                unique case (cmd_opcode)
                    OPC_INTERREGW: begin
                        mem_target <= TARGET_SPECIAL;
                        if_special_regs.write_en   <= 1'b1;
                    end

                    OPC_INTERREGR: begin
                        mem_target <= TARGET_SPECIAL;
                        if_special_regs.read_en <= 1'b1;
                        mem_read_pending <= 1'b1;
                    end

                    OPC_GLOBALREGW, OPC_JPEGREGW, OPC_DISPLAYREGW, OPC_SENSREGW: begin
                        mem_target <= TARGET_BUS;
                        if_bus.write_en <= 1'b1;
                    end

                    OPC_GLOBALREGR, OPC_JPEGREGR, OPC_DISPLAYREGR, OPC_SENSREGR: begin
                        mem_target <= TARGET_BUS;
                        if_bus.read_en <= 1'b1;
                        mem_read_pending <= 1'b1;
                    end

                    OPC_GETMEASNUM, OPC_GETMEAS, OPC_GETALLMEAS, OPC_RTMEAS: begin
                        mem_target <= TARGET_BUS;
                        if_special_regs.read_en <= 1'b1;
                        mem_read_pending <= 1'b1;
                    end

                    OPC_DISPLAYIMG: begin
                        if_special_regs.write_en   <= 1'b0;
                        if_special_regs.read_en    <= 1'b0;
                        if_bus.write_en   <= 1'b0;
                        if_bus.read_en    <= 1'b0;
                    end

                    default: begin
                        if_special_regs.write_en   <= 1'b0;
                        if_special_regs.read_en    <= 1'b0;
                        if_bus.write_en   <= 1'b0;
                        if_bus.read_en    <= 1'b0;
                    end
                endcase
            end 
        end
    end

    // ------------------------------------------------------------------------
    // DELAY and ACK handling:
    // - Wait for either timer done or read data readiness
    // - Compose ack_buf based on mem read data or write result
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_len <= 0;
            ack_idx <= 0;
            ack_sent <= 1'b0;
            all_meas_idx <= 0;
        end else begin
            ack_valid <= 1'b1;
            ack_sent <= 1'b0;
            if (state == S_DELAY) begin
                // if read completed from special regs
                if (special_read_done) begin
                    ack_len <= 11;
                    ack_buf[ 0] <= "R";
                    ack_buf[ 1] <= "E";
                    ack_buf[ 2] <= "A";
                    ack_buf[ 3] <= "D";
                    ack_buf[ 4] <= "+";
                    ack_buf[ 5] <= "O";
                    ack_buf[ 6] <= "K";
                    ack_buf[ 7] <= ":";
                    ack_buf[ 8] <= if_special_regs.read_data;
                    ack_buf[ 9] <= 8'h0D; // CR
                    ack_buf[10] <= 8'h0A; // LF
                end

                // if read completed from bus
                if (bus_read_done && opcode != OPC_GETALLMEAS) begin
                    ack_len <= 11;
                    ack_buf[ 0] <= "R";
                    ack_buf[ 1] <= "E";
                    ack_buf[ 2] <= "A";
                    ack_buf[ 3] <= "D";
                    ack_buf[ 4] <= "+";
                    ack_buf[ 5] <= "O";
                    ack_buf[ 6] <= "K";
                    ack_buf[ 7] <= ":";
                    ack_buf[ 8] <= if_bus.read_data;
                    ack_buf[ 9] <= 8'h0D; // CR
                    ack_buf[10] <= 8'h0A; // LF
                end
                
                // if write completed
                if (special_write_done || bus_write_done) begin
                    ack_len <= 11;
                    ack_buf[0] <= "W";
                    ack_buf[1] <= "R";
                    ack_buf[2] <= "I";
                    ack_buf[3] <= "T";
                    ack_buf[4] <= "E";
                    ack_buf[5] <= "+";
                    ack_buf[6] <= "O";
                    ack_buf[7] <= "K";
                    ack_buf[8] <= 8'h0D; // CR
                    ack_buf[9] <= 8'h0A; // LF
                end
                
                if (bus_read_done && opcode == OPC_GETALLMEAS) begin
                    ack_len <= 27;
                    ack_buf[0] <= "M";
                    ack_buf[1] <= "E";
                    ack_buf[2] <= "A";
                    ack_buf[3] <= "S";
                    ack_buf[4] <= "+";
                    ack_buf[5] <= "O";
                    ack_buf[6] <= "K";
                    ack_buf[7] <= ":";
                    ack_buf[8] <= if_bus.read_data;
                    ack_buf[9] <= ",";
                    all_meas_idx <= 1;
                end

                if (opc_is_displayimg()) begin
                    ack_len <= 5;
                    ack_buf[0] <= "E";
                    ack_buf[1] <= "O";
                    ack_buf[2] <= "I";
                    ack_buf[3] <= 8'h0D; // CR
                    ack_buf[4] <= 8'h0A; // LF
                end
                
                // if timer expires without read, we still go to ACK (timeout)
                if (delay_timeout) begin
                    ack_len <= 9;
                    ack_buf[0] <= "T";
                    ack_buf[1] <= "I";
                    ack_buf[2] <= "M";
                    ack_buf[3] <= "E";
                    ack_buf[4] <= "O";
                    ack_buf[5] <= "U";
                    ack_buf[6] <= "T";
                    ack_buf[7] <= 8'h0D; // CR
                    ack_buf[8] <= 8'h0A; // LF
                end
            end else if (state == S_ACK) begin
                if (opcode == OPC_GETALLMEAS && !got_all_meas) begin
                    if (all_meas_idx < ALLMEAS_SIZE) begin
                        ack_buf[8+(all_meas_idx*2)] <= if_bus.read_data;
                        ack_buf[9+(all_meas_idx*2)] <= ",";
                        all_meas_idx <= all_meas_idx + 1;
                    end else begin
                        ack_buf[]
                        got_all_meas <= 1'b1;
                    end
                end else if (ack_idx < ack_len && !tx_full) begin
                    ack_byte <= ack_buf[ack_idx];
                    ack_valid <= 1'b1;
                    ack_idx <= ack_idx + 1;
                end else if (ack_idx == ack_len) begin
                    ack_sent <= 1'b1;
                end
            end else begin
                // reset ack indices outside ACK
                ack_sent <= 1'b0;
                got_all_meas <= 1'b0;
                ack_idx <= 0;
                ack_len <= 0;
                all_meas_idx <= 0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Timer handling
    // ------------------------------------------------------------------------
    always_comb begin
        if_timer.enable = 1'b0;
        if_timer.clear = 1'b1;
        if_timer.mode = 1'b0;
        if_timer.time_count = regs_opds_time_count;
        opds_timeout = 1'b0;
        delay_timeout = 1'b0;
        if (state == S_GET_OPDS) begin
            if_timer.time_count = regs_opds_time_count;
            if_timer.enable = 1'b1;
            if_timer.clear = data_ready;
            opds_timeout = if_timer.done;
        end else if (state == S_DELAY) begin
            if_timer.time_count = regs_delay_time_count;
            if_timer.clear = 1'b0;
            delay_timeout = if_timer.done;
        end
    end


endmodule
