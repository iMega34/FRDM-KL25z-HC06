
#ifndef UART_H
#define UART_H

#include <MKL25Z4.h>
#include <fsl_port.h>
#include <fsl_uart.h>
#include <FreeRTOS.h>
#include <queue.h>

QueueHandle_t uartQueue;

/*
 * @brief Inicializa el módulo UART
 * 
 * @param uart Módulo UART a inicializar
 * @param baudRate Velocidad de transmisión en baudios
 * @param clkFreq Frecuencia del reloj del módulo UART
*/
void UART_Init_Config(UART_Type* uart, uint32_t baudRate, uint32_t clkFreq) {
    uart_config_t config;
    UART_GetDefaultConfig(&config);

    config.baudRate_Bps = baudRate;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(uart, &config, clkFreq);
    UART_EnableInterrupts(uart, kUART_RxDataRegFullInterruptEnable);

    if (uart == UART0) {
        EnableIRQ(UART0_IRQn);
    } else if (uart == UART1) {
        EnableIRQ(UART1_IRQn);
    } else if (uart == UART2) {
        EnableIRQ(UART2_IRQn);
    }
}

void UART0_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART0)) {
        data = UART_ReadByte(UART0);
        xQueueSendFromISR(uartQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART1)) {
        data = UART_ReadByte(UART1);
        xQueueSendFromISR(uartQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART2_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART2)) {
        data = UART_ReadByte(UART2);
        xQueueSendFromISR(uartQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * @brief Inicializa los pines del módulo UART
 * 
 * @param uart Módulo UART a inicializar
 * 
 * Asigna los pines correspondientes a la UART seleccionada.
 * Los modulos UART soportados son UART0, UART1 y UART2, los cuales están mapeados de la siguiente manera:
 * - UART0: Puerto A, pines 1 y 2
 * - UART1: Puerto C, pines 3 y 4
 * - UART2: Puerto E, pines 22 y 23
*/
void UART_Init_Pins(UART_Type* uart) {
    if (uart == UART0) {
        // Habilita el reloj para el puerto A
        CLOCK_EnableClock(kCLOCK_PortA);
        // Configura los pines 1 y 2 del puerto A como UART0_TX y UART0_RX
        PORT_SetPinMux(PORTA, 1U, 2U);
        PORT_SetPinMux(PORTA, 2U, 2U);
    } else if (uart == UART1) {
        // Habilita el reloj para el puerto C
        CLOCK_EnableClock(kCLOCK_PortC);
        // Configura los pines 3 y 4 del puerto C como UART1_TX y UART1_RX
        PORT_SetPinMux(PORTC, 3U, 3U);
        PORT_SetPinMux(PORTC, 4U, 3U);
    } else if (uart == UART2) {
        // Habilita el reloj para el puerto E
        CLOCK_EnableClock(kCLOCK_PortE);
        // Configura los pines 22 y 23 del puerto E como UART2_TX y UART2_RX
        PORT_SetPinMux(PORTE, 22U, 4U);
        PORT_SetPinMux(PORTE, 23U, 4U);
    }
}

#endif // UART_H
