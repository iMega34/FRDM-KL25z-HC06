
#include <stdio.h>
#include "board.h"
#include "pin_mux.h"
#include "MKL25Z4.h"
#include "peripherals.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

#include "FreeRTOS_UART.h"
#include "FreeRTOS_I2C_LCD.h"

#define I2C_MODULE                  I2C0
#define UART_MODULE                 UART2

#define QUEUE_LENGTH                16
#define MESSAGE_QUEUE_ITEM_SIZE     32
#define QUEUE_ITEM_SIZE             sizeof(char)
#define TASK_STACK_SIZE             (configMINIMAL_STACK_SIZE + 100)

/**
 * @brief Tarea que recibe los datos del módulo Bluetooth por UART desde la terminal
*/
void vTaskUARTReceiver(void *pvParameters) {
    char buffer[MESSAGE_QUEUE_ITEM_SIZE];
    char receivedChar;
    int index = 0;

    UART_Init_Config(UART_MODULE);

    while (1) {
        if (xQueueReceive(uartReceiveQueue, &receivedChar, portMAX_DELAY)) {
            if (receivedChar == '\r' || receivedChar == '\n') {
                buffer[index] = '\0';
                xQueueSendToBack(messageQueue, buffer, portMAX_DELAY);
                xQueueSendToBack(uartSendQueue, buffer, portMAX_DELAY);
                index = 0;
            } else if (receivedChar == '\b') {
                if (index > 0) index--;
            } else {
                buffer[index++] = receivedChar;
                if (index >= sizeof(buffer) - 1) {
                    index = 0;
                }
            }
        }
    }
}

/**
 * @brief Tarea que envía datos del UART desde la cola
 */
void vTaskUARTSend(void *pvParameters) {
    char receivedString[MESSAGE_QUEUE_ITEM_SIZE];

    while (1) {
        if (xQueueReceive(uartSendQueue, receivedString, portMAX_DELAY)) {
            UART_SendString(UART_MODULE, receivedString);
        }
    }
}

/**
 * @brief Tarea que recibe los mensajes de la cola de mensajes y los muestra en el LCD
*/
void vTaskLCD(void *pvParameters) {
    char receivedString[MESSAGE_QUEUE_ITEM_SIZE];

    LCD_Init_Config(I2C_MODULE);

    while (1) {
        if (xQueueReceive(messageQueue, receivedString, portMAX_DELAY)) {
            LCD_print_1st_line(receivedString);
        }
    }
}

int main() {

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    uartReceiveQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    uartSendQueue = xQueueCreate(QUEUE_LENGTH, MESSAGE_QUEUE_ITEM_SIZE);
    messageQueue = xQueueCreate(QUEUE_LENGTH, MESSAGE_QUEUE_ITEM_SIZE);
    if (uartReceiveQueue == NULL || messageQueue == NULL || uartSendQueue == NULL) {
        printf("Failed to create queues\r\n");
        while (1);
    }

    xTaskCreate(vTaskUARTReceiver, "UART Receiver", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskUARTSend, "UART Sender", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskLCD, "LCD", TASK_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
