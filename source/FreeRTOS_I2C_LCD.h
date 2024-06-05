
#ifndef FREERTOS_I2C_LCD_H
#define FREERTOS_I2C_LCD_H

#include <MKL25Z4.h>
#include <fsl_i2c.h>
#include <fsl_port.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#define I2C_BAUDRATE            100000U     // 100 kHz
#define I2C_ADDR                0x27        // Dirección del LCD

QueueHandle_t messageQueue;
SemaphoreHandle_t i2cSemaphore;

i2c_master_handle_t g_m_handle;
i2c_master_transfer_t masterXfer;
I2C_Type* master_i2c;

/*
 * @brief Inicializa el módulo I2C0
 * 
 * 
*/
void I2C_Init_Config(I2C_Type* i2c) {
    i2c_master_config_t masterConfig;
    uint32_t sourceClock;
    master_i2c = i2c;

    if (master_i2c == I2C0) {
        CLOCK_EnableClock(kCLOCK_PortC);
        PORT_SetPinMux(PORTC, 8U, 2U);   // SCL
        PORT_SetPinMux(PORTC, 9U, 2U);   // SDA
    } else if (master_i2c == I2C1) {
        CLOCK_EnableClock(kCLOCK_PortB);
        PORT_SetPinMux(PORTB, 3U, 2U);   // SCL
        PORT_SetPinMux(PORTB, 4U, 2U);   // SDA
    }

    // Configuración de I2C
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    // Inicialización de I2C
    sourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    I2C_MasterInit(master_i2c, &masterConfig, sourceClock);
    I2C_MasterTransferCreateHandle(master_i2c, &g_m_handle, NULL, NULL);

    // Creación de semáforo
    i2cSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(i2cSemaphore);
}

/*
 * @brief Envía un byte a través del bus I2C
 * 
 * @param i2c Módulo I2C a utilizar
 * @param data Byte a enviar
*/
void I2C_Write_Byte(uint8_t data) {
    if (xSemaphoreTake(i2cSemaphore, portMAX_DELAY) == pdTRUE) {
        // Envía a la dirección del esclavo (en este caso, la dirección del LCD)
        masterXfer.slaveAddress = I2C_ADDR;
        masterXfer.direction = kI2C_Write;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &data;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferDefaultFlag;

        I2C_MasterTransferBlocking(master_i2c, &masterXfer);

        // Libera el semáforo después de la transmisión
        xSemaphoreGive(i2cSemaphore);
    }
}

void LCD_Init_Config(I2C_Type* i2c) {
    I2C_Init_Config(i2c);
    // Modo 4 bits
    LCD_Command(0x02);
    // Modo 4 bits, 2 líneas, 5x7
    LCD_Command(0x28);
    // Enciende la pantalla
    LCD_Command(0x0C);
    // Mueve el cursor a la derecha
    LCD_Command(0x06);
    // Limpia la pantalla
    LCD_Command(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void LCD_Send(uint8_t data, uint8_t mode) {
    // Divide el byte en dos nibbles
    uint8_t highNibble = data & 0xF0;
    uint8_t lowNibble = (data << 4) & 0xF0;

    I2C_Write_Byte(highNibble | mode | 0x08);   // Enviar el nibble alto
    I2C_Write_Byte(highNibble | mode | 0x0C);   // Enviar habilitación alta
    I2C_Write_Byte(highNibble | mode | 0x08);   // Enviar habilitación baja

    I2C_Write_Byte(lowNibble | mode | 0x08);    // Enviar el nibble bajo
    I2C_Write_Byte(lowNibble | mode | 0x0C);    // Enviar habilitación alta
    I2C_Write_Byte(lowNibble | mode | 0x08);    // Enviar habilitación baja
}

void LCD_Command(uint8_t command) {
    LCD_Send(command, 0x00);
}

void LCD_Data(uint8_t data) {
    LCD_Send(data, 0x01);
}

void LCD_print_1st_line(char* str) {
    // Mueve el cursor al inicio de la primera línea
    LCD_Command(0x80);
    // Limpia la primera línea
    for (int i = 0; i < 16; i++) { LCD_Data(' '); }
    LCD_Command(0x80);
    // Imprime la cadena de caracteres
    while (*str) { LCD_Data(*str++); }
}

void LCD_print_2nd_line(char* str) {
    // Mueve el cursor al inicio de la segunda línea
    LCD_Command(0xC0);
    // Limpia la segunda línea
    for (int i = 0; i < 16; i++) { LCD_Data(' '); }
    LCD_Command(0xC0);
    // Imprime la cadena de caracteres
    while (*str) { LCD_Data(*str++); }
}

void LCD_clear() {
    LCD_Command(0x01);
}

#endif // FREERTOS_I2C_LCD_H
