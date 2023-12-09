/*
 * A lot of this code was taken from analog device's code
*/

#include "ams.h"

#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#include "ams_helper.h"
#include "ltcChipSettings.h"
#include "usart.h"
#include "debug.h"

/* HELPER FUNCTIONS START */

// Use normal MD (7kHz), Discharge not permission, all channels
#define ADCV_BYTE0 0x03
#define ADCV_BYTE1 0x60

#define RDCVA_BYTE0 0x00
#define RDCVA_BYTE1 0x04

#define RDCVB_BYTE0 0x00
#define RDCVB_BYTE1 0x06

#define RDCVC_BYTE0 0x00
#define RDCVC_BYTE1 0x08

#define RDCVD_BYTE0 0x00
#define RDCVD_BYTE1 0x0a

#define WRCFG_BYTE0 0x00
#define WRCFG_BYTE1 0x01

#define RDCFG_BYTE0 0x00
#define RDCFG_BYTE1 0x02

/*!
  6804 conversion command variables. Only used in set_adc functions
*/
static uint8_t ADCV[2]; //!< Cell Voltage conversion command.
static uint8_t ADAX[2]; //!< GPIO conversion command.
static uint8_t m_batt_config[NUM_BOARDS][BATT_CONFIG_SIZE] = {0};
float cell_voltages[NUM_BOARDS][12];

static uint32_t lastWakeup_ticks = 0;

void initChipConfig(void)
{
    // memset(cell_voltage_failure, 0, NUM_BOARDS*NUM_LTC_CHIPS_PER_BOARD*VOLTAGE_BLOCKS_PER_CHIP);
	
	// memset(thermistor_failure, 0, NUM_BOARDS*THERMISTORS_PER_BOARD);
	// memset(open_wire_failure, 0, NUM_BOARDS*CELLS_PER_BOARD*sizeof(open_wire_failure_t));

	for(int board = 0; board < NUM_BOARDS; board++) {
        m_batt_config[board][0] = REFON(1) | ADC_OPT(0) | SWTRD(1); // we only care about the first register in the group
        // printf("m_batt_config[%i]: %x\n", board, m_batt_config[board][0]);// THIS IS CORRECT
	}
}

HAL_StatusTypeDef batt_write_config(void)
{
    for (int i = 0; i < NUM_BOARDS; i++){
        if (ltc_write_config(m_batt_config) != HAL_OK){
            // DEBUG_PRINT("Failed to write config to LTC chip\n");
            // GPIO_TOGGLE_ERROR_LED;
            return HAL_ERROR;
        }
    }
   
    return HAL_OK;
}

HAL_StatusTypeDef batt_verify_config(void)
{
    uint8_t configBuffer[NUM_BOARDS][BATT_CONFIG_SIZE] = {0};
    if (ltc_read_config(NUM_BOARDS, configBuffer) != HAL_OK){
        // DEBUG_PRINT("Failed to read config from LTC\n"); // causing the freeze is debug_print
        GPIO_TOGGLE_ERROR_LED;
        return HAL_ERROR;
    }

    vTaskDelay(pdMS_TO_TICKS(T_REFUP_MS));

    /* PROBLEM: same problem here. It is taking way longer than expected (40 seconds)*/
    for(int board = 0; board < NUM_BOARDS; board++) {
        for(int buff_byte = 0; buff_byte < BATT_CONFIG_SIZE; buff_byte++) {
            // printf("Read Config A, board %d, byte_val: %d, expected: %d\n", board, configBuffer[board][buff_byte], m_batt_config[board][buff_byte]);
            if(m_batt_config[board][buff_byte] != configBuffer[board][buff_byte]) {
            //     DEBUG_PRINT("ERROR: m_batt_config board: %d, buff_byte %d, mismatch", board, buff_byte);
                // DEBUG_PRINT("FAILD CONFIG MATCH\n");
                // GPIO_TOGGLE_ERROR_LED;
                return HAL_ERROR;
            }
        }
    }
    printf("CONFIG GOOD\n");
    return HAL_OK;
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
{                               // 2             2             6                2               1
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
            txBuffer[data_index] = config[current_ic-1][current_byte];
            // DEBUG_PRINT("config to be written[ic = %i][byte = %i]: %x\n", current_ic, current_byte, config[current_ic][current_byte]);
            // DEBUG_PRINT("txBuffer[%i]: %x\n", data_index, txBuffer[data_index]);
            data_index++;
        }

        // calculating PEC (double check this)
        cfg_pec = (uint16_t)pec15_calc(BATT_CONFIG_SIZE, &config[current_ic - 1][0]); // calculating the PEC for each ICs configuration register data
        txBuffer[data_index] = (uint8_t)(cfg_pec >> 8);
        txBuffer[data_index + 1] = (uint8_t)cfg_pec;
        data_index = data_index + 2;
    }

    wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.

    if (spi_tx(txBuffer, BUFF_SIZE) != HAL_OK){
        // DEBUG_PRINT("Failed to transmit data to chip\n");
        // GPIO_TOGGLE_ERROR_LED;
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

    memset(txBuffer, 0, BUFF_SIZE);
    memset(rxBuffer, 0, BUFF_SIZE);

    format_ltc_command(RDCFG_BYTE0, RDCFG_BYTE1, txBuffer);
   
    wakeup_idle();

    if (spi_write_read(txBuffer, rxBuffer, BUFF_SIZE) != HAL_OK){
        // DEBUG_PRINT("Failed to write and read from chip\n");
        // GPIO_TOGGLE_Y_LED;
        return HAL_ERROR;
    }
    
    for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++){       //executes for each LTC6804 in the daisy chain and packs the data
        //into the r_config array as well as check the received Config data
        //for any bit errors
        for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG_AND_PEC; current_byte++)
        {
            tempBuffer[current_ic][current_byte] = rxBuffer[START_OF_DATA_IDX + current_byte + (current_ic*BYTES_IN_REG_AND_PEC)];
        }

        // check PEC
        received_pec = (tempBuffer[current_ic][6]<<8) + tempBuffer[current_ic][7];
        data_pec = pec15_calc(6, &tempBuffer[current_ic][0]); // PEC calculation is correct
        
        /* Problem: Taking way longer than it should (40 seconds) to run this if statement*/
        // this is failing 
        if (received_pec != data_pec)
        {
            // printf("error occurred\n");
            GPIO_TOGGLE_Y_LED;
            // DEBUG_PRINT("PEC ERROR in read config");
            return HAL_ERROR;
        }

        // Copy data out
        for (int i = 0; i < BATT_CONFIG_SIZE; i++) {
            r_config[current_ic][i] = rxBuffer[START_OF_DATA_IDX + i + (current_ic*BYTES_IN_REG_AND_PEC)];
        }
    }

    return 0;
}

void initLTC(void)
{
    set_adc(MD_NORMAL, DCP_DISABLED, CELL_CH_ALL, AUX_CH_ALL); // initialize adcv and adax commands (doesn't write)
    wakeup_sleep(); // wake up the LTC from sleep
}

HAL_StatusTypeDef LTC6804_adcv(void)
{
    const uint8_t BUFF_SIZE = COMMAND_SIZE + PEC_SIZE;
    uint8_t txBuffer[BUFF_SIZE];
    uint16_t cmd_pec = 0;

    txBuffer[0] = ADCV[0];
    txBuffer[1] = ADCV[1];

    cmd_pec = pec15_calc(COMMAND_SIZE, ADCV);
    txBuffer[2] = (uint8_t)(cmd_pec >> 8);
    txBuffer[3] = (uint8_t)(cmd_pec);

    wakeup_idle();

    if (spi_tx(txBuffer, BUFF_SIZE) != HAL_OK){
        // DEBUG_PRINT("Failed to transmit data to LTC in ADCV\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

/*!*******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command

Command Code:
-------------

|command  |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/
void set_adc(uint8_t MD, //ADC Mode
             uint8_t DCP, //Discharge Permit
             uint8_t CH, //Cell Channels to be measured
             uint8_t CHG //GPIO Channels to be measured
            )
{
    uint8_t md_bits;

    md_bits = (MD & 0x02) >> 1;
    ADCV[0] = md_bits + 0x02;
    md_bits = (MD & 0x01) << 7;
    ADCV[1] =  md_bits + 0x60 + (DCP<<4) + CH;

    md_bits = (MD & 0x02) >> 1;
    ADAX[0] = md_bits + 0x04;
    md_bits = (MD & 0x01) << 7;
    ADAX[1] = md_bits + 0x60 + CHG ;
}

/* HELPER FUNCTIONS END */

HAL_StatusTypeDef amsInit(void)
{
    AMS_CONT_CLOSE;

    initChipConfig();

    initLTC();
    
    if (batt_write_config() != HAL_OK) {
        // DEBUG_PRINT("Failed to write config to LTC chip. Stopping AMS Init\n");
        return HAL_ERROR;
    }

    if (batt_verify_config() != HAL_OK) {
        return HAL_ERROR;
        // DEBUG_PRINT("Failed to verify config on LTC chip. Stopping AMS init\n");
    }

    return HAL_OK;
}

HAL_StatusTypeDef LTC6804_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                                   uint8_t total_ic, //the number of ICs in the
                                   uint8_t *data //An array of the unparsed cell codes
                                   )
{
    const uint8_t DATA_LENGTH = COMMAND_SIZE + PEC_SIZE + ((VOLTAGE_BLOCK_SIZE + PEC_SIZE) * total_ic);
    uint8_t txBuffer[DATA_LENGTH];
    uint8_t cmdByteLow, cmdByteHigh;

    if (reg == 1)     //1: RDCVA
    {
        cmdByteHigh = 0x04;
        cmdByteLow = 0x00;
    }
    else if (reg == 2) //2: RDCVB
    {
        cmdByteHigh = 0x06;
        cmdByteLow = 0x00;
    }
    else if (reg == 3) //3: RDCVC
    {
        cmdByteHigh = 0x08;
        cmdByteLow = 0x00;
    }
    else if (reg == 4) //4: RDCVD
    {
        cmdByteHigh = 0x0A;
        cmdByteLow = 0x00;
    } else {
		// DEBUG_PRINT("Attempt to access unkown voltage block\n");
		return HAL_ERROR;
	}
    
    format_ltc_command(cmdByteLow, cmdByteHigh, txBuffer);

    wakeup_idle();

    spi_write_read(txBuffer, data, DATA_LENGTH);

    return HAL_OK;
}

HAL_StatusTypeDef readCellVoltages(uint8_t total_ic, // the number of ICs in the system
                                   float cell_codes[][12] // Array of the parsed cell codes
                                   )
{
    const uint8_t NUM_RX_BYT = 8;
    uint8_t RX_BUFFER_SIZE = (COMMAND_SIZE + PEC_SIZE + (VOLTAGE_BLOCK_SIZE + PEC_SIZE) * NUM_BOARDS);
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    uint16_t adc_val = 0;
    uint16_t received_pec = 0;
    uint16_t data_pec = 0;
    uint8_t data_counter = 0; //data counter
    float temp_voltage = 0.0;

    for (uint8_t cell_reg = 1; cell_reg < 5; cell_reg++) //executes once for each of the LTC6804 cell voltage groups
    {
        data_counter = START_OF_DATA_IDX;
        LTC6804_rdcv_reg(cell_reg, total_ic, rxBuffer); //Reads a single Cell voltage group

        for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++) // executes for every LTC6804 in the daisy chain
        {
            // current_ic is used as the IC counter

            for (uint8_t current_cell = 0; current_cell < CELL_IN_REG; current_cell++) // This loop parses the read back data into cell voltages, it
            {
                // loops once for each of the 3 cell voltage codes in the register

                adc_val = rxBuffer[data_counter] + (rxBuffer[data_counter + 1] << 8); // Each cell code is received as two bytes and is combined to
                temp_voltage = ((float)adc_val) / VOLTAGE_REGISTER_COUNTS_PER_VOLT;
                // create the parsed cell voltage code

                cell_codes[current_ic][current_cell  + ((cell_reg - 1) * CELL_IN_REG)] = temp_voltage;
                data_counter = data_counter + 2; // Because cell voltage codes are two bytes the data counter
                // must increment by two for each parsed cell code
            }
            received_pec = (rxBuffer[data_counter] << 8) + rxBuffer[data_counter + 1]; // The received PEC for the current_ic is transmitted as the 7th and 8th
            // after the 6 cell voltage data bytes
            data_pec = pec15_calc(VOLTAGE_BLOCK_SIZE, &rxBuffer[START_OF_DATA_IDX + ((VOLTAGE_BLOCK_SIZE + PEC_SIZE) * current_ic)]);
            // DEBUG_PRINT("expected: 0x%x\n", data_pec);
            // DEBUG_PRINT("received: 0x%x\n", received_pec);
            /* !! PROBLEM: the if statement below take 42 seconds to compute (or it may just be freezing FreeRTOS) before the error statement is printed*/
            if (received_pec != data_pec)
            {
                printf("wrong PEC in readCellVoltages");
                // DEBUG_PRINT("PEC Error in readCellVoltages\n");
                return HAL_ERROR;
            }
            data_counter = data_counter + 2; // Because the transmitted PEC code is 2 bytes long the data_counter
            // must be incremented by 2 bytes to point to the next ICs cell voltage data
        }
    }

    return HAL_OK;
}



void amsTask(void *pvParameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // DEBUG_PRINT("ams task starting\n");
    HAL_StatusTypeDef ret = HAL_ERROR;
    for (int num_tries = 0; num_tries < START_NUM_TRIES; num_tries++) {
        ret = amsInit();
        if(ret == HAL_OK) {
            break;
        }
        // vTaskDelay(pdMS_TO_TICKS(7)); // this is needed if wakeup_idle is removed from all config functions
    }
    if (ret != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        if (LTC6804_adcv() != HAL_OK){
            printf("failed to start ADC (adcv)\n");
            // DEBUG_PRINT("Failed to send ADCV command in amsTask\n");
            Error_Handler();
        }

        // vTaskDelay(pdMS_TO_TICKS(2)); // testing, 30 ms is good. 10 ms seems to be the lowest we can go
        // // ^ vTaskDelay might be causing issues due to inaccuracies 
        // delay_us(300); // specs say 2.3 MS

        // vTaskDelay(pdMS_TO_TICKS(3));

        if (readCellVoltages(NUM_BOARDS, cell_voltages) != HAL_OK) {
            // DEBUG_PRINT("Failed to read cell voltages and temperatures!\n");
            printf("failed to read cell voltages RDCV\n");
            Error_Handler();
        }
        // printf("main AMS loop is running\n");
        // printCellVoltages(0);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(AMS_TASK_PERIOD_MS));
    }
}

void printCellVoltages(uint8_t board)
{
    // for (int i = 0; i < CELLS_PER_BOARD; i++) {
    //     printf("Cell Voltage[%i]: %f\n", i, cell_voltages[board][i]);
    // }

    printf("Cell Voltage[1]: %f\n", cell_voltages[board][1]);
}