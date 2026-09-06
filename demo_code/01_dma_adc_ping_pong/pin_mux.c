/*
 * P3 pin mux: UART0 console (from P0) and the ADC0_SE23 analog input on
 * PTE30 (same pin P1 used). No pin for the TPM0 trigger, on purpose: it
 * fires the ADC through an internal SIM mux (Section 7 of the manual),
 * not through a physical TPM output pin, so there is nothing to mux for it.
 */
#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

#define PIN1_IDX 1u
#define PIN2_IDX 2u
#define PIN30_IDX 30u

#define SOPT5_UART0TXSRC_UART_TX 0x00u
#define SOPT5_UART0RXSRC_UART_RX 0x00u

void BOARD_InitPins(void)
{
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortE);

    /* UART0 debug console. */
    PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt2); /* PTA1 = UART0_RX */
    PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt2); /* PTA2 = UART0_TX */
    SIM->SOPT5 = ((SIM->SOPT5 & (~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK))) |
                  SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX) | SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX));

    /* ADC0_SE23 analog input header pin, same channel P1 used. */
    PORT_SetPinMux(PORTE, PIN30_IDX, kPORT_PinDisabledOrAnalog); /* PTE30 = ADC0_SE23 */
}
