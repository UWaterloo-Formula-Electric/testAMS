#ifndef AMS_H
#define AMS_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"
#include "ltcChipSettings.h"

#define START_NUM_TRIES (3)
#define AMS_TASK_PERIOD_MS 300 // should be 100 ms
#define T_IDLE_MS            5 // Time for SPI bus to go to idle state (5.5 ms)
#define T_REFUP_MS           6 // Data sheet says 3.5 ms?? won't work if any lower than 6

#define ADC_OPT(en) ((en) << 0) // Since we're using the normal 7kHz mode
#define SWTRD(en) ((en) << 1) // We're not using the software time
#define REFON(en) ((en) << 2)

#define BATT_CONFIG_SIZE 6    // 6 bytes in config register group (1 register = 1 byte)
#define START_OF_DATA_IDX 4
#define COMMAND_SIZE 2
#define PEC_SIZE 2
#define CELL_IN_REG 3
#define JUNK_SIZE 1
#define VOLTAGE_BLOCK_SIZE 6 // 6 bytes per group
#define AUX_BLOCK_SIZE 6 // 6 bytes per group
#define CELL_VOLTAGE_SIZE_BYTES 2 // 2 bytes form one cell voltage
#define THERMISTOR_CHIP 0 

#define MAX_ADC_CHECKS 10

#define VOLTAGE_REGISTER_COUNTS_PER_VOLT 10000 // 1 LSB is 100uV

/*!

 |MD| Dec  | ADC Conversion Model|
 |--|------|---------------------|
 |01| 1    | Fast            |
 |10| 2    | Normal        |
 |11| 3    | Filtered          |
*/
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3


/*!
|CH | Dec  | Channels to convert |
|---|------|---------------------|
|000| 0    | All Cells       |
|001| 1    | Cell 1 and Cell 7   |
|010| 2    | Cell 2 and Cell 8   |
|011| 3    | Cell 3 and Cell 9   |
|100| 4    | Cell 4 and Cell 10  |
|101| 5    | Cell 5 and Cell 11  |
|110| 6    | Cell 6 and Cell 12  |
*/

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6


/*!

  |CHG | Dec  |Channels to convert   |
  |----|------|----------------------|
  |000 | 0    | All GPIOS and 2nd Ref|
  |001 | 1    | GPIO 1           |
  |010 | 2    | GPIO 2               |
  |011 | 3    | GPIO 3           |
  |100 | 4    | GPIO 4           |
  |101 | 5    | GPIO 5         |
  |110 | 6    | Vref2            |
*/

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6

//uint8_t CHG = 0; //!< aux channels to be converted
/*!****************************************************
 \brief Controls if Discharging transitors are enabled
 or disabled during Cell conversions.

|DCP | Discharge Permitted During conversion |
|----|----------------------------------------|
|0   | No - discharge is not permitted         |
|1   | Yes - discharge is permitted           |

********************************************************/
#define DCP_DISABLED 0
#define DCP_ENABLED 1


void initChipConfig(void);

int batt_spi_wakeup(bool sleeping);

HAL_StatusTypeDef batt_write_config(void);

HAL_StatusTypeDef amsInit(void);

void set_adc(uint8_t MD, uint8_t DCP, uint8_t CH, uint8_t CHG);

HAL_StatusTypeDef LTC6804_adcv(void);

HAL_StatusTypeDef LTC6804_rdcv_reg(uint8_t reg, uint8_t total_ic, uint8_t *data);

HAL_StatusTypeDef readCellVoltages(uint8_t total_ic, float cell_codes[][12]);

// void LTC6804_adax();


// uint8_t LTC6804_rdcv(uint8_t reg, uint8_t total_ic, uint16_t cell_codes[][12]);

// void LTC6804_rdcv_reg(uint8_t reg, uint8_t nIC, uint8_t *data);

// int8_t LTC6804_rdaux(uint8_t reg, uint8_t nIC, uint16_t aux_codes[][6]);

// void LTC6804_rdaux_reg(uint8_t reg, uint8_t nIC,uint8_t *data);

HAL_StatusTypeDef batt_verify_config(void);
HAL_StatusTypeDef ltc_write_config(uint8_t config[NUM_BOARDS][BATT_CONFIG_SIZE]);
HAL_StatusTypeDef ltc_read_config(uint8_t total_ic, uint8_t r_config[][6]);

void printCellVoltages(uint8_t board);

#endif // AMS_H