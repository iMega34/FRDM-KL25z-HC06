
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

#define QUEUE_LENGTH 64
#define QUEUE_ITEM_SIZE sizeof(char)
#define MESSAGE_QUEUE_ITEM_SIZE 64
#define TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 100)

void vTaskBluetooth(void *pvParameters) {
    char receivedChar;
    char buffer[64];
    int index = 0;

    while (1) {
        if (xQueueReceive(uartQueue, &receivedChar, portMAX_DELAY)) {
            if (receivedChar == '\r' || receivedChar == '\n') {
                buffer[index] = '\0';
                if (index > 0) xQueueSendToBack(messageQueue, buffer, portMAX_DELAY);
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

void vTaskLCD(void *pvParameters) {
    char receivedString[64];

    LCD_Init_Config(I2C0);

    while (1) {
        if (xQueueReceive(messageQueue, receivedString, portMAX_DELAY)) {
            int line = receivedString[0] - '0';
            char *text = receivedString + 2;

            if (line == 1) {
                LCD_print_1st_line(text);
            } else if (line == 2) {
                LCD_print_2nd_line(text);
            }
        }
    }
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    UART_Init_Config(UART2, 9600, CLOCK_GetFreq(UART2_CLK_SRC));
    UART_Init_Pins(UART2);

    uartQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    messageQueue = xQueueCreate(QUEUE_LENGTH, MESSAGE_QUEUE_ITEM_SIZE);
    if (uartQueue == NULL || messageQueue == NULL) {
        printf("Failed to create UART or Message queue\r\n");
        while (1);
    }

    xTaskCreate(vTaskBluetooth, "Bluetooth", TASK_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(vTaskLCD, "LCD", TASK_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();

    for (;;);
}
