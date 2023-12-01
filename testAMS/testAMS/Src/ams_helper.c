// includes all helper functions used in ams.c
#include "ams_helper.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

/* delay function for wakeup */
void delay_us(uint32_t time_us)
{
	__HAL_TIM_SetCounter(&DELAY_TIMER,0);
	__HAL_TIM_SetAutoreload(&DELAY_TIMER,0xffff);
	HAL_TIM_Base_Start(&DELAY_TIMER);
	while(DELAY_TIMER_INSTANCE->CNT < time_us);
	HAL_TIM_Base_Stop(&DELAY_TIMER);
}

void wakeup_idle(){
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    delay_us(2);
    // verify on the scope
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
}

void wakeup_sleep()
{
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1)); // verify is 1 ms on the scope. Also check tolerance on datasheet. 
    // if needed, use hardware timer
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef spi_tx(uint8_t *txBuffer, uint32_t length)
{
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET); // chip select pin
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&ISO_SPI_HANDLE, txBuffer, length, HSPI_TIMEOUT);
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef spi_write_read(uint8_t * tdata, uint8_t * rbuffer, unsigned int len)
{   // check on scope
    //Make sure rbuffer is large enough for the sending command + receiving bytes
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&ISO_SPI_HANDLE, tdata, rbuffer, len, HSPI_TIMEOUT);
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
    return status;
}

/* Taken from Analog Device */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate a PEC
                   )
{
    uint16_t remainder,addr;

    remainder = 16; // initialize the PEC
    for (uint8_t i = 0; i<len; i++){ // loops for each byte in data array
        addr = ((remainder>>7)^data[i])&0xff; // calculate PEC table address
        remainder = (remainder<<8)^crc15Table[addr];
    }

    return(remainder*2); // The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

/* Communication always start with the command followed by its PEC. Then data. */
void format_ltc_command(uint8_t cmdByteLow, uint8_t cmdByteHigh, uint8_t *txBuffer)
{
    uint16_t cmd_pec;
    txBuffer[0] = cmdByteLow;
    txBuffer[1] = cmdByteHigh;

    cmd_pec = pec15_calc(COMMAND_SIZE, txBuffer);
    txBuffer[2] = (uint8_t)(cmd_pec >> 8);
    txBuffer[3] = (uint8_t)(cmd_pec);
}
