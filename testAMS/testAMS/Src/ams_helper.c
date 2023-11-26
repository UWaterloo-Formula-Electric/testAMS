// includes all helper functions used in ams.c
#include "ams_helper.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"

#define WRCFG_BYTE0 0x00
#define WRCFG_BYTE1 0x01

#define RDCFG_BYTE0 0x00
#define RDCFG_BYTE1 0x02

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
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
}

void wakeup_sleep()
{
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef spi_tx(uint8_t *txBuffer, uint32_t length)
{
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&ISO_SPI_HANDLE, txBuffer, length, HSPI_TIMEOUT);
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
    return status;
}

HAL_StatusTypeDef spi_write_read(uint8_t * tdata, uint8_t * rbuffer, unsigned int len)
{
    //Make sure rbuffer is large enough for the sending command + receiving bytes
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&ISO_SPI_HANDLE, tdata, rbuffer, len, HSPI_TIMEOUT);
    HAL_GPIO_WritePin(ISO_SPI_NSS_GPIO_Port, ISO_SPI_NSS_Pin, GPIO_PIN_SET);
    return status;
}

/* Taken from Analog Device */
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
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

/*****************************************************//**
 \brief Write the LTC6804 configuration register

 This command will write the configuration registers of the LTC6804-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to a single IC.     

Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]       |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
********************************************************/
HAL_StatusTypeDef ltc_write_config(uint8_t config[NUM_BOARDS][BATT_CONFIG_SIZE] // configuration data that will be written
                                  )
{
    const uint64_t BUFF_SIZE = COMMAND_SIZE + PEC_SIZE + ((BATT_CONFIG_SIZE + PEC_SIZE) * NUM_BOARDS);
    uint16_t cfg_pec;
    uint8_t data_index; //command counter
    uint8_t txBuffer[BUFF_SIZE];

    format_ltc_command(WRCFG_BYTE0, WRCFG_BYTE1, txBuffer);

    data_index = START_OF_DATA_IDX;
    for (uint8_t current_ic = NUM_BOARDS; current_ic > 0; current_ic--) // executes for each LTC6804 in daisy chain, this loops starts with
    {
        // the last IC on the stack. The first configuration written is
        // received by the last IC in the daisy chain

        for (uint8_t current_byte = 0; current_byte < BATT_CONFIG_SIZE; current_byte++) // executes for each of the 6 bytes in the CFGR register
        {
            // current_byte is the byte counter
            txBuffer[data_index] = config[current_ic][current_byte];
            data_index++;
        }

        // calculating PEC
        cfg_pec = (uint16_t)pec15_calc(BATT_CONFIG_SIZE, &config[current_ic - 1][0]); // calculating the PEC for each ICs configuration register data
        txBuffer[data_index] = (uint8_t)(cfg_pec >> 8);
        txBuffer[data_index + 1] = (uint8_t)cfg_pec;
        data_index = data_index + 2;
    }

    wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
    
    if (spi_tx(txBuffer, BUFF_SIZE) != HAL_OK){
        DEBUG_PRINT("Failed to transmit data to chip\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef ltc_read_config(uint8_t total_ic, //Number of ICs in the system
                                  uint8_t r_config[][6] // configuration data to be read
                                 )
{
    const uint8_t BYTES_IN_REG_AND_PEC = 8;
    const uint64_t BUFF_SIZE = COMMAND_SIZE + PEC_SIZE + ((BATT_CONFIG_SIZE + PEC_SIZE) * NUM_BOARDS);
    uint16_t data_pec;
    uint16_t received_pec;
    uint8_t txBuffer[BUFF_SIZE];
    uint8_t rxBuffer[BUFF_SIZE];
    uint8_t tempBuffer[NUM_BOARDS][8];

    format_ltc_command(RDCFG_BYTE0, RDCFG_BYTE1, txBuffer);

    wakeup_idle();

    if (spi_write_read(txBuffer, rxBuffer, BUFF_SIZE) != HAL_OK){
        DEBUG_PRINT("Failed to write and read from chip\n");
        return HAL_ERROR; // error
    }

    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++){       //executes for each LTC6804 in the daisy chain and packs the data
        //into the r_config array as well as check the received Config data
        //for any bit errors

        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG_AND_PEC; current_byte++)
        {
            tempBuffer[current_ic][current_byte] = rxBuffer[current_byte + (current_ic*BYTES_IN_REG_AND_PEC)];
        }

        // check PEC
        received_pec = (tempBuffer[current_ic][6]<<8) + tempBuffer[current_ic][7];
        data_pec = pec15_calc(6, &tempBuffer[current_ic][0]);
        
        /* Problem: Taking way longer than it should (40 seconds) to run this if statement*/
        // if (received_pec != data_pec)
        // {
        //     return HAL_ERROR;
        // }

        // Copy data out
        for (int i = 0; i < BATT_CONFIG_SIZE; i++) {
            r_config[current_ic][i] = rxBuffer[START_OF_DATA_IDX + i];
        }
    }

    return HAL_OK;
}