// ---------------------------------------------------------
// Module: TIMER
// Description: Timer module for generate scheduled pulses
// Author: André Lamego
// Date: 2025-04-17
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef TIMER_SV
`define TIMER_SV

module timer #(
    parameter CLOCK_F       = 50_000_000, // Clock frequency, default 50MHz
    parameter MAX_TIME_US   = 10_000_000  // Maximum time in µs (10s)
) (
    tmr_if if_t,

    input  logic        clk,
    input  logic        rst_n
);

    localparam int micro_counter = $rtoi(CLOCK_F / 1000000); // Clock cycles per microsecond

    logic [7:0] counter_us; // Counter register to get 1µs
    logic [19:0] counter;   // Counter register to get the time in µs
    logic done; // Done signal to indicate the end of counting
    logic reload; // Reload signal for auto-reload mode
    logic prev_done; // Previous state of done signal for edge detection
    logic rising_done; // Rising edge detection for done signal

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            counter_us <= 0;
            done <= 0;
        end else begin
            if (if_t.clear || reload) begin
                counter <= 0;
                counter_us <= 0;
                done <= 0;
            end else if (if_t.enable) begin
                if ({ ($bits(micro_counter)-$bits(counter_us ))'('0),counter_us } < micro_counter-1) begin
                    counter_us <= counter_us + 1;
                end else begin
                    counter_us <= 0;
                    if ({ ($bits(if_t.time_count)-$bits(counter ))'('0),counter } < if_t.time_count - 1 && { ($bits(MAX_TIME_US)-$bits(counter ))'('0),counter } < MAX_TIME_US) begin
                        counter <= counter + 1;
                    end else begin
                        done <= 1; // Signal that the time has elapsed
                        counter <= 0; // Reset the counter after done
                    end
                end
            end else begin
                done <= 0; // Reset done signal when not enabled
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_done <= 0;
            rising_done <= 0;
        end else begin
            prev_done <= done;
            rising_done <= done && !prev_done; // Detect rising edge
        end
    end

    always_comb begin
        reload = if_t.mode && rising_done;
        if_t.done = rising_done;
    end
        
endmodule

`endif // TIMER_SV
