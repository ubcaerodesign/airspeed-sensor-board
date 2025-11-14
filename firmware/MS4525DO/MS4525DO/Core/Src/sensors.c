/*
 * sensors.c
 *
 *  Created on: Nov 13, 2025
 *      Author: ellay
 */
#include "sensors.h"
#include "bno055.h"
#include "ms4525do.h"

extern bno055_vector_t bno_quaternion, bno_accel, bno_gyro;
extern struct MS4525DO_t ms4525do;
sensor_data_t sensor_data = {0};

void populate_sensor_data(sensor_data_t *sensor_data){
	sensor_data->accel_x = -bno_accel.y;
	sensor_data->accel_y = -bno_accel.x;
	sensor_data->accel_z = -bno_accel.z;

	sensor_data->gyro_x = -bno_gyro.y*DEG2RAD;
	sensor_data->gyro_y = -bno_gyro.x*DEG2RAD;
	sensor_data->gyro_z = -bno_gyro.z*DEG2RAD;

	sensor_data->q0 = -bno_quaternion.y;
	sensor_data->q1 = -bno_quaternion.x;
	sensor_data->q2 = -bno_quaternion.z;
	sensor_data->q3 = bno_quaternion.w;

	sensor_data->pressure_psi = ms4525do.processed_data.pressure_psi;
	sensor_data->temperature_c = ms4525do.processed_data.temperature_C;
	sensor_data->airspeed_mps = ms4525do.processed_data.airspeed_mps;

	sensor_data->time = HAL_GetTick();
}



void save_sensor_data_sd(sensor_data_t *sensor_data);





