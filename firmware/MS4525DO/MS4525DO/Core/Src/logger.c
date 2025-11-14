/*
 * bare_logger.c
 *
 *  Created on: Nov 2025
 *      Author: You
 */

#include "logger.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#define LOG_BUFFER_SIZE 256
#define FINAL_LOG_BUFFER_SIZE 400

// Default log level
#ifdef DEBUG
static log_level_t current_log_level = LOG_DEBUG;
#else
static log_level_t current_log_level = LOG_INFO;
#endif

static UART_HandleTypeDef *log_uart = NULL; // optional UART output

void logger_init(UART_HandleTypeDef *huart, log_level_t level) {
    log_uart = huart;
    current_log_level = level;
}


void log_message(log_level_t level, const char *file, int line, const char *format, ...) {
    if (level < current_log_level) return;

    char log_buffer[LOG_BUFFER_SIZE];

    va_list args;
    va_start(args, format);
    vsnprintf(log_buffer, LOG_BUFFER_SIZE, format, args);
    va_end(args);

    // timestamp
    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "[%lu] ", HAL_GetTick());

    // log level string
    const char *level_str;
    switch(level) {
        case LOG_DEBUG: level_str = "[DEBUG] "; break;
        case LOG_INFO:  level_str = "[INFO] "; break;
        case LOG_WARN:  level_str = "[WARN] "; break;
        case LOG_ERROR: level_str = "[ERROR] "; break;
        case LOG_FATAL: level_str = "[FATAL] "; break;
        default:        level_str = "[LOG] "; break;
    }

    // filename only
    const char *filename = strrchr(file, '/');
    filename = filename ? filename + 1 : file;

    // final formatted log line
    char final_log[FINAL_LOG_BUFFER_SIZE];
    snprintf(final_log, FINAL_LOG_BUFFER_SIZE, "%s %s %s (%s:%d)\r\n",
             timestamp, level_str, log_buffer, filename, line);

    // ---- UART output ----
#if LOG_TO_UART
    if (log_uart != NULL) {
        HAL_UART_Transmit(log_uart, (uint8_t*)final_log, strlen(final_log), HAL_MAX_DELAY);
    }
#endif

    // ---- SD Card output ----
#if LOG_TO_SD
    sd_append("LOG.TXT", final_log);
#endif
}
