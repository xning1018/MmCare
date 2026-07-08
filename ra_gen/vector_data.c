/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = iic_master_rxi_isr, /* IIC1 RXI (Receive data full) */
            [1] = iic_master_txi_isr, /* IIC1 TXI (Transmit data empty) */
            [2] = iic_master_tei_isr, /* IIC1 TEI (Transmit end) */
            [3] = iic_master_eri_isr, /* IIC1 ERI (Transfer error) */
            [4] = sci_uart_rxi_isr, /* SCI2 RXI (Received data full) */
            [5] = sci_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [6] = sci_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [7] = sci_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [8] = gpt_counter_overflow_isr, /* GPT0 COUNTER OVERFLOW (Overflow) */
            [9] = sci_uart_rxi_isr, /* SCI4 RXI (Received data full) */
            [10] = sci_uart_txi_isr, /* SCI4 TXI (Transmit data empty) */
            [11] = sci_uart_tei_isr, /* SCI4 TEI (Transmit end) */
            [12] = sci_uart_eri_isr, /* SCI4 ERI (Receive error) */
            [13] = sci_uart_rxi_isr, /* SCI9 RXI (Received data full) */
            [14] = sci_uart_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [15] = sci_uart_tei_isr, /* SCI9 TEI (Transmit end) */
            [16] = sci_uart_eri_isr, /* SCI9 ERI (Receive error) */
            [17] = sci_uart_rxi_isr, /* SCI3 RXI (Received data full) */
            [18] = sci_uart_txi_isr, /* SCI3 TXI (Transmit data empty) */
            [19] = sci_uart_tei_isr, /* SCI3 TEI (Transmit end) */
            [20] = sci_uart_eri_isr, /* SCI3 ERI (Receive error) */
            [21] = agt_int_isr, /* AGT0 INT (AGT interrupt) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_IIC1_RXI), /* IIC1 RXI (Receive data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_IIC1_TXI), /* IIC1 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_IIC1_TEI), /* IIC1 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_IIC1_ERI), /* IIC1 ERI (Transfer error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_SCI2_RXI), /* SCI2 RXI (Received data full) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TXI), /* SCI2 TXI (Transmit data empty) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TEI), /* SCI2 TEI (Transmit end) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_SCI2_ERI), /* SCI2 ERI (Receive error) */
            [8] = BSP_PRV_IELS_ENUM(EVENT_GPT0_COUNTER_OVERFLOW), /* GPT0 COUNTER OVERFLOW (Overflow) */
            [9] = BSP_PRV_IELS_ENUM(EVENT_SCI4_RXI), /* SCI4 RXI (Received data full) */
            [10] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TXI), /* SCI4 TXI (Transmit data empty) */
            [11] = BSP_PRV_IELS_ENUM(EVENT_SCI4_TEI), /* SCI4 TEI (Transmit end) */
            [12] = BSP_PRV_IELS_ENUM(EVENT_SCI4_ERI), /* SCI4 ERI (Receive error) */
            [13] = BSP_PRV_IELS_ENUM(EVENT_SCI9_RXI), /* SCI9 RXI (Received data full) */
            [14] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI), /* SCI9 TXI (Transmit data empty) */
            [15] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI), /* SCI9 TEI (Transmit end) */
            [16] = BSP_PRV_IELS_ENUM(EVENT_SCI9_ERI), /* SCI9 ERI (Receive error) */
            [17] = BSP_PRV_IELS_ENUM(EVENT_SCI3_RXI), /* SCI3 RXI (Received data full) */
            [18] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TXI), /* SCI3 TXI (Transmit data empty) */
            [19] = BSP_PRV_IELS_ENUM(EVENT_SCI3_TEI), /* SCI3 TEI (Transmit end) */
            [20] = BSP_PRV_IELS_ENUM(EVENT_SCI3_ERI), /* SCI3 ERI (Receive error) */
            [21] = BSP_PRV_IELS_ENUM(EVENT_AGT0_INT), /* AGT0 INT (AGT interrupt) */
        };
        #endif