#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "logger.h"
#include "bno055.h"
#include "sensors.h"
#include "fatfs.h"
#include "sd_card.h"

#define RAD_TO_DEG(rad) ((rad) * (180.0 / M_PI))

#define ACCEL_SYNC_TIME 500
#define ACCEL_SYNC_THRESHOLD 0.15
#define GRAVITY 9.80665

#define accelScale 100.0
#define tempScale 1.0
#define angularRateScale 16.0
#define eulerScale 16.0
#define magScale 16.0
#define quaScale ((double)(1<<14)) // 2^14

#define BNO055_I2C_ADDR_HI 0x29
#define BNO055_I2C_ADDR_LO 0x28
#define BNO055_I2C_ADDR    BNO055_I2C_ADDR_LO
#define EEPROM_I2C_ADDR 0x50

/**
 * Registers
 */
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00
#define BNO055_ACC_ID 0x01
#define BNO055_MAG_ID 0x02
#define BNO055_GYRO_ID 0x03
#define BNO055_SW_REV_ID_LSB 0x04
#define BNO055_SW_REV_ID_MSB 0x05
#define BNO055_BL_REV_ID 0x06
#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_DATA_X_LSB 0x08
#define BNO055_ACC_DATA_X_MSB 0x09
#define BNO055_ACC_DATA_Y_LSB 0x0A
#define BNO055_ACC_DATA_Y_MSB 0x0B
#define BNO055_ACC_DATA_Z_LSB 0x0C
#define BNO055_ACC_DATA_Z_MSB 0x0D
#define BNO055_MAG_DATA_X_LSB 0x0E
#define BNO055_MAG_DATA_X_MSB 0x0F
#define BNO055_MAG_DATA_Y_LSB 0x10
#define BNO055_MAG_DATA_Y_MSB 0x11
#define BNO055_MAG_DATA_Z_LSB 0x12
#define BNO055_MAG_DATA_Z_MSB 0x13
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_GYR_DATA_X_MSB 0x15
#define BNO055_GYR_DATA_Y_LSB 0x16
#define BNO055_GYR_DATA_Y_MSB 0x17
#define BNO055_GYR_DATA_Z_LSB 0x18
#define BNO055_GYR_DATA_Z_MSB 0x19
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_EUL_HEADING_MSB 0x1B
#define BNO055_EUL_ROLL_LSB 0x1C
#define BNO055_EUL_ROLL_MSB 0x1D
#define BNO055_EUL_PITCH_LSB 0x1E
#define BNO055_EUL_PITCH_MSB 0x1F
#define BNO055_QUA_DATA_W_LSB 0x20
#define BNO055_QUA_DATA_W_MSB 0x21
#define BNO055_QUA_DATA_X_LSB 0x22
#define BNO055_QUA_DATA_X_MSB 0x23
#define BNO055_QUA_DATA_Y_LSB 0x24
#define BNO055_QUA_DATA_Y_MSB 0x25
#define BNO055_QUA_DATA_Z_LSB 0x26
#define BNO055_QUA_DATA_Z_MSB 0x27
#define BNO055_LIA_DATA_X_LSB 0x28
#define BNO055_LIA_DATA_X_MSB 0x29
#define BNO055_LIA_DATA_Y_LSB 0x2A
#define BNO055_LIA_DATA_Y_MSB 0x2B
#define BNO055_LIA_DATA_Z_LSB 0x2C
#define BNO055_LIA_DATA_Z_MSB 0x2D
#define BNO055_GRV_DATA_X_LSB 0x2E
#define BNO055_GRV_DATA_X_MSB 0x2F
#define BNO055_GRV_DATA_Y_LSB 0x30
#define BNO055_GRV_DATA_Y_MSB 0x31
#define BNO055_GRV_DATA_Z_LSB 0x32
#define BNO055_GRV_DATA_Z_MSB 0x33
#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_ACC_OFFSET_X_MSB 0x56
#define BNO055_ACC_OFFSET_Y_LSB 0x57
#define BNO055_ACC_OFFSET_Y_MSB 0x58
#define BNO055_ACC_OFFSET_Z_LSB 0x59
#define BNO055_ACC_OFFSET_Z_MSB 0x5A
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_MAG_OFFSET_X_MSB 0x5C
#define BNO055_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_MAG_OFFSET_Z_MSB 0x60
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_GYR_OFFSET_X_MSB 0x62
#define BNO055_GYR_OFFSET_Y_LSB 0x63
#define BNO055_GYR_OFFSET_Y_MSB 0x64
#define BNO055_GYR_OFFSET_Z_LSB 0x65
#define BNO055_GYR_OFFSET_Z_MSB 0x66
#define BNO055_ACC_RADIUS_LSB 0x67
#define BNO055_ACC_RADIUS_MSB 0x68
#define BNO055_MAG_RADIUS_LSB 0x69
#define BNO055_MAG_RADIUS_MSB 0x6A

#define BNO055_PAGE_ID 0x07
#define BNO055_ACC_CONFIG 0x08
#define BNO055_MAG_CONFIG 0x09
#define BNO055_GYRO_CONFIG_0 0x0A
#define BNO055_GYRO_CONFIG_1 0x0B
#define BNO055_ACC_SLEEP_CONFIG 0x0C
#define BNO055_GYR_SLEEP_CONFIG 0x0D
#define BNO055_INT_MSK 0x0F
#define BNO055_INT_EN 0x10
#define BNO055_ACC_AM_THRES 0x11
#define BNO055_ACC_INT_SETTINGS 0x12
#define BNO055_ACC_HG_DURATION 0x13
#define BNO055_ACC_HG_THRESH 0x14
#define BNO055_ACC_NM_THRESH 0x15
#define BNO055_ACC_NM_SET 0x16
#define BNO055_GYR_INT_SETTINGS 0x17
#define BNO055_GYR_HR_X_SET 0x18
#define BNO055_GYR_DUR_X 0x19
#define BNO055_GYR_HR_Y_SET 0x1A
#define BNO055_GYR_DUR_Y 0x1B
#define BNO055_GYR_HR_Z_SET 0x1C
#define BNO055_GYR_DUR_Z 0x1D
#define BNO055_GYR_AM_THRESH 0x1E
#define BNO055_GYR_AM_SET 0x1F

/** Operation Mode Settings **/
typedef enum {
  OPERATION_MODE_CONFIG = 0X00,
  OPERATION_MODE_ACCONLY = 0X01,
  OPERATION_MODE_MAGONLY = 0X02,
  OPERATION_MODE_GYRONLY = 0X03,
  OPERATION_MODE_ACCMAG = 0X04,
  OPERATION_MODE_ACCGYRO = 0X05,
  OPERATION_MODE_MAGGYRO = 0X06,
  OPERATION_MODE_AMG = 0X07,
  OPERATION_MODE_IMUPLUS = 0X08,
  OPERATION_MODE_COMPASS = 0X09,
  OPERATION_MODE_M4G = 0X0A,
  OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
  OPERATION_MODE_NDOF = 0x0C
} bno055_opmode_t;

/** BNO055 power settings */
typedef enum {
  POWER_MODE_NORMAL = 0X00,
  POWER_MODE_LOWPOWER = 0X01,
  POWER_MODE_SUSPEND = 0X02
} bno055_powermode_t;

/** Remap settings **/
typedef enum {
  REMAP_CONFIG_P0 = 0x21,
  REMAP_CONFIG_P1 = 0x24, // default
  REMAP_CONFIG_P2 = 0x24,
  REMAP_CONFIG_P3 = 0x21,
  REMAP_CONFIG_P4 = 0x24,
  REMAP_CONFIG_P5 = 0x21,
  REMAP_CONFIG_P6 = 0x21,
  REMAP_CONFIG_P7 = 0x24
} bno055_axis_remap_config_t;

/** Remap Signs **/
typedef enum {
  REMAP_SIGN_P0 = 0x04,
  REMAP_SIGN_P1 = 0x00, // default
  REMAP_SIGN_P2 = 0x06,
  REMAP_SIGN_P3 = 0x02,
  REMAP_SIGN_P4 = 0x03,
  REMAP_SIGN_P5 = 0x01,
  REMAP_SIGN_P6 = 0x07,
  REMAP_SIGN_P7 = 0x05
} bno055_axis_remap_sign_t;


typedef enum {
  BNO055_VECTOR_ACCELEROMETER = 0x08,  // Default: m/s²
  BNO055_VECTOR_MAGNETOMETER = 0x0E,   // Default: uT
  BNO055_VECTOR_GYROSCOPE = 0x14,      // Default: deg/s
  BNO055_VECTOR_EULER = 0x1A,          // Default: degrees
  BNO055_VECTOR_QUATERNION = 0x20,     // No units
  BNO055_VECTOR_LINEARACCEL = 0x28,    // Default: m/s²
  BNO055_VECTOR_GRAVITY = 0x2E         // Default: m/s²
} bno055_vector_type_t;

/** Structure to represent offsets **/
typedef struct {
  int16_t accel_offset_x; /**< x acceleration offset */
  int16_t accel_offset_y; /**< y acceleration offset */
  int16_t accel_offset_z; /**< z acceleration offset */

  int16_t mag_offset_x; /**< x magnetometer offset */
  int16_t mag_offset_y; /**< y magnetometer offset */
  int16_t mag_offset_z; /**< z magnetometer offset */

  int16_t gyro_offset_x; /**< x gyroscrope offset */
  int16_t gyro_offset_y; /**< y gyroscrope offset */
  int16_t gyro_offset_z; /**< z gyroscrope offset */

  int16_t accel_radius; /**< acceleration radius */

  int16_t mag_radius; /**< magnetometer radius */
} bno055_offsets_t;

/** Calibration States **/
typedef struct {
  uint8_t sys;
  uint8_t gyro;
  uint8_t mag;
  uint8_t accel;
} bno055_calibration_state_t;

/** Structure to store BNO055 sensor data **/
typedef struct {
  double w;
  double x;
  double y;
  double z;
} bno055_vector_t;

void bno055_set_page(uint8_t page);
void bno055_reset();
void bno055_use_external_crystal(bool state);
bno055_opmode_t bno055_get_operation_mode();
void bno055_set_operation_mode(bno055_opmode_t mode);
void bno055_set_operation_mode_config();
void bno055_set_operation_mode_ndof();
void bno055_set_axis_remap(bno055_axis_remap_config_t remapcode);
void bno055_set_axis_sign(bno055_axis_remap_sign_t remapsign);
void bno055_init();

bool bno055_get_calibration_state(bno055_calibration_state_t *calState);
void bno055_get_calibration_data(bno055_offsets_t *calData, bno055_calibration_state_t *calState);
void bno055_set_calibration_data(bno055_offsets_t *calData);
void bno055_save_calibration_data_sd(bno055_offsets_t *calData);
void bno055_load_calibration_data_sd(void);
void bno055_display_calibration_data(bno055_offsets_t *calData);
void bno055_calibration_routine();

void bno055_get_vector(uint8_t vec, bno055_vector_t *xyz);
void bno055_get_vector_accelerometer(bno055_vector_t *xyz);
void bno055_get_vector_magnetometer(bno055_vector_t *xyz);
void bno055_get_vector_gyroscope(bno055_vector_t *xyz);
void bno055_get_vector_euler(bno055_vector_t *xyz);
void bno055_get_vector_linear_accel(bno055_vector_t *xyz);
void bno055_get_vector_gravity(bno055_vector_t *xyz);
void bno055_get_vector_quaternion(bno055_vector_t *xyz);

void bno055_get_vector_euler_from_vector_quaternion(bno055_vector_t *quaternion, bno055_vector_t *euler);
void bno055_get_magnetic_heading
(bno055_vector_t *refQuaternion, bno055_vector_t *refGravity, bno055_vector_t *quaternion, double *heading);
void bno055_get_references(bno055_vector_t *refQuaternion, bno055_vector_t *refGravity);

void bno055_inverse_quaternion(bno055_vector_t *q);
void bno055_multiply_quaternion(bno055_vector_t *q1, bno055_vector_t *q2, bno055_vector_t *q3);
void bno055_inverse_matrix(double matrix[3][3]);
void bno055_quaternion_to_matrix(bno055_vector_t *q, double matrix[3][3]);
void bno055_matrix_vector_multiply(double matrix[3][3], bno055_vector_t *refGravity, bno055_vector_t *newGravity);
void bno055_normalize(bno055_vector_t *q);
void bno055_quaternion_to_euler(bno055_vector_t *q, bno055_vector_t *euler);

void bno055_flush();
#endif
