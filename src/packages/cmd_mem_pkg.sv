// ---------------------------------------------------------
// Module: Command Memory Package
// Description: Defines commands used to configure BLE modules
// as peripherals and enable advertising (HM-10 and HC-05)
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------

`ifndef CMD_MEM_PKG_SV
`define CMD_MEM_PKG_SV

package cmd_mem_pkg;

    // Commands for HM-10 BLE Module (Peripheral Configuration)
    localparam string HM10_CMD_AT        = $sformatf("AT%c\n", 8'h0D);          // Test AT command
    localparam string HM10_CMD_NAME      = $sformatf("AT+NAME%c\n", 8'h0D);     // Set device name
    localparam string HM10_CMD_PIN       = $sformatf("AT+PIN%c\n", 8'h0D);      // Set PIN code
    localparam string HM10_CMD_BAUD      = $sformatf("AT+BAUD%c\n", 8'h0D);     // Set baud rate
    localparam string HM10_CMD_ROLE      = $sformatf("AT+ROLE0%c\n", 8'h0D);    // Set role to peripheral
    localparam string HM10_CMD_RESET     = $sformatf("AT+RESET%c\n", 8'h0D);    // Reset module
    localparam string HM10_CMD_ADV       = $sformatf("AT+ADVEN%c\n", 8'h0D);    // Enable advertising

    // Commands for HC-05 Bluetooth Module (Peripheral Configuration)
    localparam string HC05_CMD_AT        = $sformatf("AT%c\n", 8'h0D);          // Test AT command
    localparam string HC05_CMD_NAME      = $sformatf("AT+NAME=%c\n", 8'h0D);     // Set device name
    localparam string HC05_CMD_PIN       = $sformatf("AT+PSWD=%c\n", 8'h0D);     // Set PIN code
    localparam string HC05_CMD_BAUD      = $sformatf("AT+UART=%c\n", 8'h0D);     // Set baud rate
    localparam string HC05_CMD_ROLE      = $sformatf("AT+ROLE=0%c\n", 8'h0D);   // Set role to slave (peripheral)
    localparam string HC05_CMD_RESET     = $sformatf("AT+RESET%c\n", 8'h0D);    // Reset module

endpackage // cmd_mem_pkg

`endif // CMD_MEM_PKG_SV
