#include "debug.h"
#include "bsp.h"
#include "stdio.h"
#include "string.h"
#include "task.h"

// Buffer to receive uart characters (1 byte)
uint8_t uartDMA_rxBuffer = '\000';

uint8_t isUartOverCanEnabled = 0;

QueueHandle_t printQueue;
QueueHandle_t uartRxQueue;

// Note: printf should only be used for printing before RTOS starts, and error
// cases where rtos has probably failed. (If used after rtos starts, it may
// cause errors in calling non-reentrant hal functions)
int _write(int file, char* data, int len) {
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)data, len, UART_PRINT_TIMEOUT);
    return len;
}

HAL_StatusTypeDef uartStartReceiving(UART_HandleTypeDef *huart) {
    __HAL_UART_FLUSH_DRREGISTER(huart); // Clear the buffer to prevent overrun
    return HAL_UART_Receive_DMA(huart, &uartDMA_rxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR( uartRxQueue, &uartDMA_rxBuffer, &xHigherPriorityTaskWoken );

    if( xHigherPriorityTaskWoken )
    {
        portYIELD();
    }
}


HAL_StatusTypeDef debugInit()
{
    printQueue = xQueueCreate(PRINT_QUEUE_LENGTH, PRINT_QUEUE_STRING_SIZE);
    if (!printQueue)
    {
        return HAL_ERROR;
    }

    uartRxQueue = xQueueCreate(UART_RX_QUEUE_LENGTH, 1);
    if (!uartRxQueue)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void printTask(void *pvParameters)
{
    char buffer[PRINT_QUEUE_STRING_SIZE] = {0};

    for ( ;; )
    {
        if (xQueueReceive(printQueue, (uint8_t*) buffer, portMAX_DELAY) == pdTRUE)
        {
            uint64_t len = strlen(buffer);
            HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*) buffer, len, UART_PRINT_TIMEOUT);

            // if(isUartOverCanEnabled)
            // {
            //     // send message length
            //     UartOverCanRX = len;
            //     sendCAN_UartOverCanRx();
                
            //     // Send CLI response to the CAN
            //     const uint16_t chunkLen = 4; // bytes per CAN message
			// 	for(uint16_t a = 0; a < len; a += chunkLen)
			// 	{
            //         UartOverCanRX = buffer[a];
            //         for (uint16_t i = 0; i < chunkLen && a + i < len; i++)
            //         {
            //             UartOverCanRX |= buffer[a + i] << (i * 8);
            //         }
			// 		sendCAN_UartOverCanRx();
			// 	}
            // }
        }
    }
}