/*
 * sd_card.h
 *
 *  Created on: Apr 6, 2025
 *      Author: chant
 */

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_
#include "fatfs.h"
#include "config.h"


void save_sensors_sd(sensor_data_t *sensor_data);
FRESULT sd_log_line_to_file(const char* filename, const char* log_line);
FRESULT sd_append(const char *filename, const char *line);

#endif
