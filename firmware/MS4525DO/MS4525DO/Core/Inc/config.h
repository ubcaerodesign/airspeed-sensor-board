/*
 * debugging.h
 *
 *  Created on: Mar 21, 2025
 *      Author: ella
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#define PRINTF_OVERLOAD //let printf print to serial
#define AIRSPEED_CAN_ID 0x300
#define FC_CAN_ID 0x100

typedef struct {
    double accel_x;
    double accel_y;
    double accel_z;

    double gyro_x;
    double gyro_y;
    double gyro_z;

    double mag_x;
    double mag_y;
    double mag_z;

    float q0;
    float q1;
    float q2;
    float q3;

    double pressure_psi;
    double temperature_c;
    double airspeed_mps;

    double time;

} sensor_data_t;

#endif /* INC_CONFIG_H_ */
