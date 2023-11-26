#include "main.h"
#include "FreeRTOS.h"
#include "stdio.h"
#include "string.h"
#include "generalErrorHandler.h"
#include "debug.h"

// Reset the debug uart
// This is done to clear the UART in case it is being used by the debug task,
// that way we can send an error message
HAL_StatusTypeDef resetUART()
{
    if (HAL_UART_DeInit(&DEBUG_UART_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UART_Init(&DEBUG_UART_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Flag to ensure we only trigger error handling once
void _handleError(char *file, int line)
{
  const char errorStringFile[] = "Error!: File ";
  const char errorStringLine[] = " line ";
  char lineNumberString[10];

  taskDISABLE_INTERRUPTS();

  if (resetUART() != HAL_OK) {
      // Can't really do anything else
    while(1)
    {
    }
  }

  HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)errorStringFile), strlen(errorStringFile), 1000);
  HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)file), strlen(file), 1000);
  HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)errorStringLine), strlen(errorStringLine), 1000);
  snprintf(lineNumberString, sizeof(lineNumberString), "%d\n", line);
  HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)lineNumberString), strlen(lineNumberString), 1000);

  while(1)
  {

  }

}
