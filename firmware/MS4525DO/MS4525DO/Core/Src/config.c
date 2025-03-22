/*
 * debuggin.c
 *
 *  Created on: Mar 21, 2025
 *      Author: ellay
 */

#include "config.h"
#include <stdio.h>
#include "main.h"

#ifdef PRINTF_OVERLOAD
// Use the handle for the UART you configured (e.g., huart1)
extern UART_HandleTypeDef huart1;
int _write(int file, char *data, int len) {
    // Transmit data via UART
    HAL_UART_Transmit(&huart1, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
#endif
