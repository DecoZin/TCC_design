// ---------------------------------------------------------
// Module: Command Memory Package
// Description: Defines for commands and registers used to
// setup the BLE module
// Author: André Lamego
// Date: 2025-05-06
// ---------------------------------------------------------
`timescale 1ns / 100ps

`ifndef EI_MEM_PKG_SV
`define EI_MEM_PKG_SV
    
package ei_mem_pkg;

    // Registers (43 registers in total)
    typedef enum logic [5:0] { // Adjust the bit width as needed (64 registers possible)
        EIR_TEST,
        EIR_SLP_MODE,
        EIR_SLP_TMR0,
        EIR_SLP_TMR1,
        EIR_SLP_TMR2,
        EIR_PROGRAMMER,
        EIR_ADV_INT,
        EIR_ADV_TMR0,
        EIR_ADV_TMR1,
        EIR_ADV_TMR2,
        EIR_CENTRAL_MAC0,
        EIR_CENTRAL_MAC1,
        EIR_CENTRAL_MAC2,
        EIR_CENTRAL_MAC3,
        EIR_CENTRAL_MAC4,
        EIR_CENTRAL_MAC5,
        EIR_DVC_NAME0,
        EIR_DVC_NAME1,
        EIR_DVC_NAME2,
        EIR_DVC_NAME3,
        EIR_DVC_NAME4,
        EIR_DVC_NAME5,
        EIR_DVC_NAME6,
        EIR_DVC_NAME7,
        EIR_DVC_PIN0,
        EIR_DVC_PIN1,
        EIR_DVC_PIN2,
        EIR_DVC_PIN3,
        EIR_DVC_PIN4,
        EIR_DVC_PIN5,
        EIR_BAUD_RATE,
        EIR_CONN_TMR0,
        EIR_CONN_TMR1,
        EIR_CONN_TMR2,
        EIR_OPDS_TMR0,
        EIR_OPDS_TMR1,
        EIR_OPDS_TMR2,
        EIR_ACK_TMR0,
        EIR_ACK_TMR1,
        EIR_ACK_TMR2,
        EIR_RX_DATA,
        EIR_TX_DATA,
        EIR_DO_DATA,
        EIR_ERROR
    } ei_regs_t;

endpackage // ei_mem_pkg

`endif // EI_MEM_PKG_SV
