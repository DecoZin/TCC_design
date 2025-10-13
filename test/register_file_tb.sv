// ---------------------------------------------------------
// Module: Register File Testbench
// Description: Testbench for the Register File module
// Author: André Lamego
// Date: 2025-05-08
// ---------------------------------------------------------
`timescale 1ns / 1ps

module register_file_tb;
    import ei_mem_pkg::*;

    logic clk   = 0;
    logic rst_n = 1;

    localparam real CLK_FREQ = 50_000_000; // Clock frequency in Hz
    localparam real CLK_PERIOD = 1e9 / CLK_FREQ; // in ns
    localparam DATA_WIDTH = 8;
    localparam DATA_DEPTH = 64;

    // Timer values for various registers
    logic [23:0] adv_tmr = 24'd5_000_000;
    logic [23:0] conn_tmr = 24'd10_000_000;
    logic [23:0] opds_tmr = 24'd1_000_000;
    logic [23:0] ack_tmr = 24'd10_000;

    // Interface instantiations
    regs_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_regs_inst();

    regs_int_if #(
        .DATA_DEPTH(DATA_DEPTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) if_top_link();

    // DUT instantiation
    register_file #(
        .DATA_WIDTH(DATA_WIDTH),
        .DATA_DEPTH(DATA_DEPTH)
    ) dut (
        .if_regs_inst(if_regs_inst.slave),
        .if_top_link(if_top_link.slave),
        .clk(clk),
        .rst_n(rst_n)
    );

    // Clock generation
    initial fork
        generate_clock(clk, CLK_PERIOD / 2);
    join_none

    // Test procedure
    initial begin
        // Reset values
        if_top_link.regi = '{default: '0};  // Default values for all registers
        if_top_link.mode_mask = '0;  // No register is read-only initially

        // Assign specific values to some registers
        if_top_link.regi[EIR_TEST] = 8'hAA;
        if_top_link.mode_mask[EIR_TEST] = 1'b1; // Read-Only for EIR_TEST

        if_top_link.regi[EIR_PROGRAMMER] = 8'h00;
        if_top_link.regi[EIR_ADV_INT] = "3";

        if_top_link.regi[EIR_ADV_TMR0] = adv_tmr[7:0];
        if_top_link.regi[EIR_ADV_TMR1] = adv_tmr[15:8];
        if_top_link.regi[EIR_ADV_TMR2] = adv_tmr[23:16];

        if_top_link.regi[EIR_DVC_NAME0] = "O";
        if_top_link.regi[EIR_DVC_NAME1] = "C";
        if_top_link.regi[EIR_DVC_NAME2] = "V";
        if_top_link.regi[EIR_DVC_NAME3] = "1";

        if_top_link.regi[EIR_DVC_PIN0] = "1";
        if_top_link.regi[EIR_DVC_PIN1] = "2";
        if_top_link.regi[EIR_DVC_PIN2] = "3";
        if_top_link.regi[EIR_DVC_PIN3] = "4";
        if_top_link.regi[EIR_DVC_PIN4] = "5";
        if_top_link.regi[EIR_DVC_PIN5] = "6";

        if_top_link.regi[EIR_BAUD_RATE] = "0";
        if_top_link.mode_mask[EIR_BAUD_RATE] = 1'b1; // Read-Only

        if_top_link.regi[EIR_CONN_TMR0] = conn_tmr[7:0];
        if_top_link.regi[EIR_CONN_TMR1] = conn_tmr[15:8];
        if_top_link.regi[EIR_CONN_TMR2] = conn_tmr[23:16];

        if_top_link.regi[EIR_OPDS_TMR0] = opds_tmr[7:0];
        if_top_link.regi[EIR_OPDS_TMR1] = opds_tmr[15:8];
        if_top_link.regi[EIR_OPDS_TMR2] = opds_tmr[23:16];

        if_top_link.regi[EIR_ACK_TMR0] = ack_tmr[7:0];
        if_top_link.regi[EIR_ACK_TMR1] = ack_tmr[15:8];
        if_top_link.regi[EIR_ACK_TMR2] = ack_tmr[23:16];
    end

    int error_cnt = 0;

    // Reset and operation test
    initial begin
        DEBUG_INFO("TB", "Testbench started.");
        
        // Reset the DUT
        #10;
        rst_n = 0;
        if_regs_inst.write_en = '0;
        if_regs_inst.read_en = '0;
        if_regs_inst.addr = '0;
        if_regs_inst.write_data = '0;
        #(CLK_PERIOD);
        rst_n = 1;
        #(CLK_PERIOD);
        
        // Testing write operation
        DEBUG_INFO("TB", "Testing write operation.");
        if_regs_inst.write_en = 1'b1;
        for (int i = 0; i < DATA_DEPTH; i++) begin
            if_regs_inst.addr = i;
            if_regs_inst.write_data = i;
            #(CLK_PERIOD);
        end

        // Testing read operation
        DEBUG_INFO("TB", "Testing read operation.");
        if_regs_inst.write_en = 1'b0;
        if_regs_inst.read_en = 1'b1;
        for (int i = 0; i < DATA_DEPTH; i++) begin
            if_regs_inst.addr = i;
            #(CLK_PERIOD);
            if (if_regs_inst.read_data != i && if_top_link.mode_mask[i] == 1'b0) begin
                DEBUG_ERR("READ", $sformatf("Error: Expected %0d, got %0d at address %0d.", i, if_regs_inst.read_data, i));
                error_cnt = error_cnt + 1;
            end else if (if_top_link.mode_mask[i] == 1'b1) begin
                DEBUG_INFO("READ", $sformatf("Read-Only: Address %0d, got %0d.", i, if_regs_inst.read_data));
            end else begin
                DEBUG_INFO("READ", $sformatf("Read: Address %0d, got %0d.", i, if_regs_inst.read_data));
            end
        end
        if (error_cnt > 0) begin
            DEBUG_ERR("TB", $sformatf("Error count: %0d", error_cnt));
            DEBUG_FAIL();
        end else begin
            DEBUG_INFO("TB", "No errors detected.");
            DEBUG_PASS();
        end
        // End simulation
        #20;
        $finish;
    end

    // Generate waveform dump for the simulation
    initial begin
        $dumpfile("waveform.vcd");      // Specify the VCD file name
        $dumpvars(0, register_file_tb); // Dump all signals in the testbench
    end

endmodule
