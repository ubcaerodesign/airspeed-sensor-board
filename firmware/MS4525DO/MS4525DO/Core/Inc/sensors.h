/*
 * sensors.h
 *
 *  Created on: Nov 13, 2025
 *      Author: ellay
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdint.h>
#include "config.h"

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

// IMU / BNO055 data

void populate_sensor_data(sensor_data_t *sensor_data);


#endif /* INC_SENSORS_H_ */
