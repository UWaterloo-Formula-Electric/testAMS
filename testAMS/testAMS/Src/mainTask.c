/**
  *****************************************************************************
  * @file    mainTaskEntry.c
  * @author  Richard Matthews
  * @brief   Module containing main task, which is the default task for all
  * boards. It currently blinks the debug LED to indicate the firmware is running
  *****************************************************************************
  */

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#define MAIN_TASK_PERIOD 1000

#define DEBUG_LED_PORT LED_B_GPIO_Port
#define DEBUG_LED_PIN LED_B_Pin

void mainTaskFunction(void const * argument)
{
    // DEBUG_PRINT("Starting up!!\n");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char message[50] = "Hello world \n";

    while (1) {
        HAL_GPIO_TogglePin(DEBUG_LED_PORT, DEBUG_LED_PIN);
        HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), 100); 
        printf("Test printf\n");
        vTaskDelayUntil(&xLastWakeTime, MAIN_TASK_PERIOD);
    }
}
