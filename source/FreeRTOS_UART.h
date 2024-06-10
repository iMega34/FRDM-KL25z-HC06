
#ifndef UART_H
#define UART_H

#include <MKL25Z4.h>
#include <fsl_port.h>
#include <fsl_uart.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

#define MAX_CHUNK_LENGTH        (20 * sizeof(uint8_t))

SemaphoreHandle_t uartMutex;
QueueHandle_t uartSendQueue;
QueueHandle_t uartReceiveQueue;

/**
 * Inicializa el módulo UART
 */
void UART_Init_Config(UART_Type* uart) {
    uart_config_t config;
    uint32_t uartClkSrcFreq;

    if (uart == UART0) {
        CLOCK_EnableClock(kCLOCK_PortA);
        PORT_SetPinMux(PORTA, 1U, 2U);
        PORT_SetPinMux(PORTA, 2U, 2U);
    } else if (uart == UART1) {
        CLOCK_EnableClock(kCLOCK_PortC);
        PORT_SetPinMux(PORTC, 3U, 3U);
        PORT_SetPinMux(PORTC, 4U, 3U);
    } else if (uart == UART2) {
        CLOCK_EnableClock(kCLOCK_PortE);
        PORT_SetPinMux(PORTE, 22U, 4U);
        PORT_SetPinMux(PORTE, 23U, 4U);
    }

    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 9600;
    config.enableTx = true;
    config.enableRx = true;

    uartClkSrcFreq = CLOCK_GetFreq(kCLOCK_BusClk);
    UART_Init(uart, &config, uartClkSrcFreq);
    UART_EnableInterrupts(uart, kUART_RxDataRegFullInterruptEnable);

    uartMutex = xSemaphoreCreateMutex();
    if (uartMutex == NULL) {
        printf("Failed to create UART Mutex\r");
        while (1);
    }

    if (uart == UART0) {
        EnableIRQ(UART0_IRQn);
    } else if (uart == UART1) {
        EnableIRQ(UART1_IRQn);
    } else if (uart == UART2) {
        EnableIRQ(UART2_IRQn);
    }
}

/**
 * @brief Envía una cadena de caracteres a través del módulo UART
 *
 * @param uart Módulo de UART a utilizar
 * @param str Cadena de caracteres a enviar
 */
void UART_SendString(UART_Type* uart, const char* str) {
    if (xSemaphoreTake(uartMutex, portMAX_DELAY)) {
        char response[128];
        sprintf(response, "Comando recibido: %s\r", str);

        size_t offset = 0;
        size_t stringLength = strlen(response);

        while (offset < stringLength) {
            size_t chunkLength = (stringLength - offset < MAX_CHUNK_LENGTH)
                ? stringLength - offset
                : MAX_CHUNK_LENGTH;

            UART_WriteBlocking(uart, (uint8_t*) (response + offset), chunkLength);
            offset += chunkLength;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        xSemaphoreGive(uartMutex);
    }
}

/*
 * @brief Inicializa la rutina de interrupción para el módulo UART0
 */
void UART0_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART0)) {
        data = UART_ReadByte(UART0);
        xQueueSendFromISR(uartReceiveQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * @brief Inicializa la rutina de interrupción para el módulo UART1
 */
void UART1_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART1)) {
        data = UART_ReadByte(UART1);
        xQueueSendFromISR(uartReceiveQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*
 * @brief Inicializa la rutina de interrupción para el módulo UART2
 */
void UART2_IRQHandler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t data;

    if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(UART2)) {
        data = UART_ReadByte(UART2);
        xQueueSendFromISR(uartReceiveQueue, &data, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#endif // UART_H
