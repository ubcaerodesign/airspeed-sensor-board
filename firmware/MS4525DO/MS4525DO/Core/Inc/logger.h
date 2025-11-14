/*
 * logger.h
 *
 *  Created on: Nov 2025
 *      Author: You
 *
 */

#ifndef INC_LOGGER_H
#define INC_LOGGER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "sd_card.h"

// Enable UART/SD logging as needed
#define LOG_TO_UART 1
#define LOG_TO_SD   1

// Log levels
typedef enum {
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL
} log_level_t;

/**
 * @brief Initialize the logger.
 * @param huart Pointer to UART handle for UART logging (can be NULL if not used)
 * @param level Default log level
 */
void logger_init(UART_HandleTypeDef *huart, log_level_t level);

/**
 * @brief Log a message with level, file, and line information
 * @param level Log level
 * @param file __FILE__ macro
 * @param line __LINE__ macro
 * @param format printf-style format string
 */
void log_message(log_level_t level, const char *file, int line, const char *format, ...);

/**
 * @brief Convenience macro for logging with automatic file + line
 */
#define LOG(level, fmt, ...) log_message(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

// Macro to make logging easier
#define LOG_MESSAGE(level, format, ...) \
    do { \
        log_message(level, __FILE__, __LINE__, format, ##__VA_ARGS__); \
    } while(0)


#endif // BARE_LOGGER_H
