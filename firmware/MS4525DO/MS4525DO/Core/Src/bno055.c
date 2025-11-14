/*
 * bno055.c
 *
 *  Created on: Nov 13, 2025
 *      Author: ellay
 */



/**
 * GLOBALS
 */
#include "stm32f1xx_hal.h"
#include "main.h"
#include "bno055.h"

#define CAL_TIMEOUT_MS 15000

double theta_F_old = 0, phi_F_old = 0;
double theta = 0, phi = 0;
/*
 * PRIVATE FUNCTIONS
 */

static HAL_StatusTypeDef _bno055_write_register(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef _bno055_read_register(uint8_t reg, uint8_t *data, uint8_t len);

// fast calibration-on-boot helper declaration
static void bno055_fast_calibration_on_boot(void);

/**
 * @brief Write to sensor
 * note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
static HAL_StatusTypeDef _bno055_write_register(uint8_t reg, uint8_t data) {
	uint8_t txdata[2] = {reg, data};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(BNO055_I2C, BNO055_I2C_ADDR << 1, txdata, sizeof(txdata), HAL_MAX_DELAY);

    return status;

}



/**
 * @brief Read from sensor
 *  note: i2c HAL API automatically handles the I2C write/read bit for you.
 */
static HAL_StatusTypeDef _bno055_read_register(uint8_t reg, uint8_t *data, uint8_t len) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(BNO055_I2C, BNO055_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);

    return status;
}


void bno055_set_page(uint8_t page) { _bno055_write_register(BNO055_PAGE_ID, page); }

void bno055_reset() {
	HAL_Delay(650);
	_bno055_write_register(BNO055_SYS_TRIGGER, 0x20);
	HAL_Delay(700);
}

void bno055_use_external_crystal(bool state) {
	bno055_set_page(0);
	uint8_t tmp = 0;
	_bno055_read_register(BNO055_SYS_TRIGGER, &tmp, 1);
	tmp |= (state == true) ? 0x80 : 0x0;
	_bno055_write_register(BNO055_SYS_TRIGGER, tmp);
}

bno055_opmode_t bno055_get_operation_mode() {
	bno055_opmode_t mode;
	_bno055_read_register(BNO055_OPR_MODE, &mode, 1);
	return mode;
}

void bno055_set_operation_mode(bno055_opmode_t mode) {
	_bno055_write_register(BNO055_OPR_MODE, mode);
	if (mode == OPERATION_MODE_CONFIG) {
	HAL_Delay(19);
	} else {
	HAL_Delay(7);
	}
}

void bno055_set_operation_mode_config() {
	bno055_set_operation_mode(OPERATION_MODE_CONFIG);
}

void bno055_set_operation_mode_ndof() {
	//bno055_set_operation_mode(OPERATION_MODE_NDOF);
	//bno055_set_operation_mode(OPERATION_MODE_IMUPLUS);
	bno055_set_operation_mode(OPERATION_MODE_NDOF);
}

void bno055_set_axis_remap(bno055_axis_remap_config_t remapcode) {
	bno055_set_operation_mode_config();
	_bno055_write_register(BNO055_AXIS_MAP_CONFIG, remapcode);
	bno055_set_operation_mode_ndof();
}

void bno055_set_axis_sign(bno055_axis_remap_sign_t remapsign) {
	bno055_set_operation_mode_config();
	_bno055_write_register(BNO055_AXIS_MAP_SIGN, remapsign);
	bno055_set_operation_mode_ndof();
}


void bno055_init() {
	bno055_reset();
	LOG_MESSAGE(LOG_INFO, "BNO055 RESET");


	uint8_t id = 0;
	_bno055_read_register(BNO055_CHIP_ID, &id, 1);
	if (id != BNO055_ID) {
		LOG_MESSAGE(LOG_INFO, "Can't find BNO055, id: 0x%02x. Please check your wiring.", id);
	}
	bno055_set_operation_mode_config();

	_bno055_write_register(BNO055_PWR_MODE, POWER_MODE_NORMAL);
	bno055_set_page(0);
	_bno055_write_register(BNO055_SYS_TRIGGER, 0x0);

	//BNO055_setOperationModeConfig();
	bno055_set_operation_mode_ndof();

	// Attempt a fast calibration on boot (load EEPROM offsets and try to finish calibration quickly while logging progress)
	bno055_fast_calibration_on_boot();

}

bool bno055_get_calibration_state(bno055_calibration_state_t *calState) {
	bno055_set_page(0);
	uint8_t cal = 0;

	_bno055_read_register(BNO055_CALIB_STAT, &cal, 1);

	calState->sys = (cal >> 6) & 0x03;
	calState->gyro = (cal >> 4) & 0x03;
	calState->accel = (cal >> 2) & 0x03;
	calState->mag = cal & 0x03;

	if((calState->sys == 3) && (calState->gyro == 3) && (calState->accel == 3) && (calState->mag == 3)){
	  return true;
	}
	/*if((calState->gyro == 3)){
	  return true;
	}*/
	else{
	  return false;
	}

}

void bno055_get_calibration_data(bno055_offsets_t *calData, bno055_calibration_state_t *calState) {
	uint8_t buffer[22];

	// Ensure the sensor is fully calibrated before reading and saving offsets
	if (!bno055_get_calibration_state(calState)) {
		// Not fully calibrated - do not read or save offsets
		return;
	}

	bno055_set_operation_mode_config(); // switch to config to read/write offset registers

	_bno055_read_register(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

	calData->accel_offset_x = ((buffer[1] << 8) | buffer[0]);
	calData->accel_offset_y = ((buffer[3] << 8) | buffer[2]);
	calData->accel_offset_z = ((buffer[5] << 8) | buffer[4]);

	calData->mag_offset_x = ((buffer[7] << 8) | buffer[6]);
	calData->mag_offset_y = ((buffer[9] << 8) | buffer[8]);
	calData->mag_offset_z = ((buffer[11] << 8) | buffer[10]);

	calData->gyro_offset_x = ((buffer[13] << 8) | buffer[12]);
	calData->gyro_offset_y = ((buffer[15] << 8) | buffer[14]);
	calData->gyro_offset_z = ((buffer[17] << 8) | buffer[16]);

	calData->accel_radius = ((buffer[19] << 8) | buffer[18]);

	calData->mag_radius = ((buffer[21] << 8) | buffer[20]);

	bno055_set_operation_mode_ndof();

	// Save new offsets into EEPROM only when sensor is fully calibrated
	bno055_save_calibration_data_sd(calData);

}

void bno055_set_calibration_data(bno055_offsets_t *calData) {
	uint8_t buffer[22]; // 22 bytes

	memcpy(buffer, calData, 22); //STM32 is little-endian so this is fine
	//uint8_t *byteData = (uint8_t *)calData; //try this some time

	bno055_set_operation_mode_config();

	for (uint8_t i=0; i < 22; i++) {
		_bno055_write_register(BNO055_ACC_OFFSET_X_LSB+i, buffer[i]);
	}

	bno055_set_operation_mode_ndof();
}

/**
 * Storing offset data starting at address 0x0000 of EEPROM
 */

void bno055_save_calibration_data_sd(bno055_offsets_t *calData) {
    // Format offsets into a CSV string
    char line[128];
    snprintf(line, sizeof(line),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
             calData->accel_offset_x, calData->accel_offset_y, calData->accel_offset_z,
             calData->mag_offset_x, calData->mag_offset_y, calData->mag_offset_z,
             calData->gyro_offset_x, calData->gyro_offset_y, calData->gyro_offset_z,
             calData->accel_radius, calData->mag_radius);

    // Append the line to the SD card
    sd_append("bno_cal.txt", line);
}
/**
 * Read offsets starting at address 0x0000 of EEPROM
 */

void bno055_load_calibration_data_sd(void) {
    FIL file;
    FRESULT res;
    char line[128];
    bno055_offsets_t calDataStruct;
    bno055_offsets_t *calData = &calDataStruct;

    // Open the calibration file
    res = f_open(&file, "bno_cal.txt", FA_READ);
    if (res != FR_OK) {
        // File doesn't exist or can't open, nothing to load
        return;
    }

    // Read the last line of the file
    while (f_gets(line, sizeof(line), &file) != NULL) {
        // loop will leave the last line in 'line'
    }
    f_close(&file);

    // Check if we actually read something
    if (strlen(line) == 0) {
        return;
    }

    // Parse CSV line into offsets safely
    int num_parsed = sscanf(line,
           "%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd",
           &calData->accel_offset_x, &calData->accel_offset_y, &calData->accel_offset_z,
           &calData->mag_offset_x, &calData->mag_offset_y, &calData->mag_offset_z,
           &calData->gyro_offset_x, &calData->gyro_offset_y, &calData->gyro_offset_z,
           &calData->accel_radius, &calData->mag_radius);

    if (num_parsed != 11) {
        // parsing failed, don't apply bad data
        return;
    }

    // Optionally display calibration data
    bno055_display_calibration_data(calData);

    // Apply calibration to the BNO055
    bno055_set_calibration_data(calData);
}


void bno055_display_calibration_data(bno055_offsets_t *calData){
    LOG_MESSAGE(LOG_INFO, "Accel: ");
    LOG_MESSAGE(LOG_INFO, "%d, %d, %d", calData->accel_offset_x,calData->accel_offset_y, calData->accel_offset_z);

    LOG_MESSAGE(LOG_INFO, "Gyro: ");
    LOG_MESSAGE(LOG_INFO,"%d, %d, %d", calData->gyro_offset_x,calData->gyro_offset_y, calData->gyro_offset_z);

    LOG_MESSAGE(LOG_INFO, "Mag: ");
    LOG_MESSAGE(LOG_INFO,"%d, %d, %d", calData->mag_offset_x,calData->mag_offset_y, calData->mag_offset_z);

    LOG_MESSAGE(LOG_INFO, "Accel Radius: ");
    LOG_MESSAGE(LOG_INFO, "%d", calData->accel_radius);

    LOG_MESSAGE(LOG_INFO, "Mag Radius: ");
    LOG_MESSAGE(LOG_INFO, "%d", calData->mag_radius);

    HAL_Delay(500);

}

void bno055_calibration_routine(){
	bno055_calibration_state_t calState;
	bno055_offsets_t calData;

	bno055_set_page(0);

	while(! bno055_get_calibration_state(&calState)){
		LOG_MESSAGE(LOG_DEBUG,"sys = %d, gyro = %d, mag = %d, accel = %d", calState.sys, calState.gyro, calState.mag, calState.accel);
		HAL_Delay(500);

	}
	LOG_MESSAGE(LOG_DEBUG, "--------CALIBRATION COMPLETE STORING INTO MEMORY--------");
	bno055_get_calibration_data(&calData, &calState);
	bno055_display_calibration_data(&calData);
	bno055_set_calibration_data(&calData);
	LOG_MESSAGE(LOG_DEBUG, "--------DONE--------");

}

void bno055_get_vector(uint8_t vec, bno055_vector_t *xyz) {
	bno055_set_page(0);
	uint8_t buffer[8];    // Quaternion needs 8 bytes

	if (vec == BNO055_VECTOR_QUATERNION)
		_bno055_read_register(vec, buffer, 8);
	else
		_bno055_read_register(vec, buffer, 6);

	double scale = 1;

	if (vec == BNO055_VECTOR_MAGNETOMETER) {
		scale = magScale;
	} else if (vec == BNO055_VECTOR_ACCELEROMETER ||
		   vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY) {
		scale = accelScale;
	} else if (vec == BNO055_VECTOR_GYROSCOPE) {
		scale = angularRateScale;
	} else if (vec == BNO055_VECTOR_EULER) {
		scale = eulerScale;
	} else if (vec == BNO055_VECTOR_QUATERNION) {
		scale = quaScale;
	}

	if (vec == BNO055_VECTOR_QUATERNION) {
		xyz->w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
		xyz->x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz->y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
		xyz->z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;
	} else {
		xyz->x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale; //heading, roll, pitch
		xyz->y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
		xyz->z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
	}
}

void bno055_get_vector_accelerometer(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_ACCELEROMETER, xyz);
}
void bno055_get_vector_magnetometer(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_MAGNETOMETER, xyz);
}
void bno055_get_vector_gyroscope(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_GYROSCOPE, xyz);
}
void bno055_get_vector_euler(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_EULER, xyz);
}
void bno055_get_vector_linear_accel(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_LINEARACCEL, xyz);
}
void bno055_get_vector_gravity(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_GRAVITY, xyz);
}
void bno055_get_vector_quaternion(bno055_vector_t *xyz) {
	bno055_get_vector(BNO055_VECTOR_QUATERNION, xyz);
}

void bno055_get_vector_euler_from_vector_quaternion(bno055_vector_t *quaternion,bno055_vector_t *euler){
	bno055_vector_t convertedQuaternion;
	bno055_vector_t changeQuaternion = {.w = 0.5, .x = 0.5, .y = 0.5, .z = 0.5};

	bno055_multiply_quaternion(&changeQuaternion, quaternion, &convertedQuaternion);

    double sq_w = convertedQuaternion.w * convertedQuaternion.w;
    double sq_x = convertedQuaternion.x * convertedQuaternion.x;
    double sq_y = convertedQuaternion.y * convertedQuaternion.y;
    double sq_z = convertedQuaternion.z * convertedQuaternion.z;

    double unit = sq_w + sq_x + sq_y + sq_z;
    double test = convertedQuaternion.x * convertedQuaternion.y + convertedQuaternion.z * convertedQuaternion.w;

    /** x = roll, y = pitch, z = yaw **/
    euler->x = atan2(2 * (convertedQuaternion.w * convertedQuaternion.x + convertedQuaternion.y * convertedQuaternion.z), sq_y + sq_w - sq_x - sq_z);
    euler->y = atan2(2 * (convertedQuaternion.w * convertedQuaternion.y - convertedQuaternion.x * convertedQuaternion.z), sq_w + sq_x - sq_y - sq_z);

    // initial conditionals are for catching singularities when converting to Euler
    if (test > 0.4999 * unit) {
    	euler->z = M_PI / 2;
    }
    else if (test < -0.4999 * unit) {
    	euler->z = -M_PI / 2;
    }
    else {
    	euler->z = asin(2 * test / unit);
    }

}

void bno055_get_magnetic_heading
(bno055_vector_t *refQuaternion, bno055_vector_t *refGravity, bno055_vector_t *quaternion, double *heading){

    double theta_M, phi_M;
    double theta_F_new, phi_F_new;
    double Xm, Ym;

    double rot_matrix[3][3] = {
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}};

    bno055_vector_t diffQuaternion, newGravity, magnetometer;

    bno055_multiply_quaternion(refQuaternion, quaternion, &diffQuaternion);
    bno055_quaternion_to_matrix(&diffQuaternion, rot_matrix);
    bno055_inverse_matrix(rot_matrix);
    bno055_matrix_vector_multiply(rot_matrix, refGravity, &newGravity);

    bno055_get_vector_magnetometer(&magnetometer);

    theta_M = atan2(newGravity.x / GRAVITY, newGravity.z / GRAVITY);
    phi_M = atan2(newGravity.y / GRAVITY, newGravity.z / GRAVITY);
    phi_F_new = 0.95 * phi_F_old + 0.05 * phi_M;
    theta_F_new = 0.95 * theta_F_old + 0.05 * theta_M;

    theta = theta * 0.95 + theta_M * 0.05;
    phi = phi * 0.95 + phi_M * 0.05;

    Xm = magnetometer.x * cos(theta) - magnetometer.y * sin(phi) * sin(theta) + magnetometer.z * cos(phi) * sin(theta);
    Ym = magnetometer.y * cos(phi) + magnetometer.z * sin(phi);

    *heading = atan2(Ym, Xm) * 180 / M_PI;

    theta_F_old = theta_F_new;
    phi_F_old = phi_F_new;

    if (*heading < 0) {
        *heading += 360;
    }

}

void bno055_get_references(bno055_vector_t *refQuaternion, bno055_vector_t *refGravity){
	bno055_vector_t initialAccel, newAccel;

	bno055_get_vector_accelerometer(&initialAccel);

	unsigned long start_test = HAL_GetTick();

	// checks if IMU is stationary within defined test duration
	while ( HAL_GetTick() - start_test <= ACCEL_SYNC_TIME ) {
		bno055_get_vector_accelerometer(&newAccel);

		// checks if acceleration components exceed defined threshold
		if ( !( abs(newAccel.x - initialAccel.x) < ACCEL_SYNC_THRESHOLD &&
				abs(newAccel.y - initialAccel.y) < ACCEL_SYNC_THRESHOLD &&
				abs(newAccel.z - initialAccel.z) < ACCEL_SYNC_THRESHOLD )) {

			// update start time and initial acceleration
			bno055_get_vector_accelerometer(&initialAccel);
			start_test = HAL_GetTick();
		}
	}

	// get reference gravity vector and quaternion
	bno055_get_vector_accelerometer(refGravity);
	bno055_get_vector_quaternion(refQuaternion);
	bno055_inverse_quaternion(refQuaternion);

}

void bno055_inverse_quaternion(bno055_vector_t *q) { //is this right lol
	q->w = q->w;
	q->x = -(q->x);
	q->y = -(q->y);
	q->z = -(q->z);
}

void bno055_multiply_quaternion(bno055_vector_t *q1, bno055_vector_t *q2, bno055_vector_t *q3) {
	 q3->w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
	 q3->x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
	 q3->y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
	 q3->z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
}

void bno055_inverse_matrix(double matrix[3][3]){
	double temp;

	// Swap elements across the diagonal
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 3; j++) {
			// Swap matrix[i][j] and matrix[j][i]
			temp = matrix[i][j];
			matrix[i][j] = matrix[j][i];
			matrix[j][i] = temp;
		}
	}
}

void bno055_quaternion_to_matrix(bno055_vector_t *q, double matrix[3][3]) {
    matrix[0][0] = 2 * (q->w * q->w + q->x * q->x) - 1;
    matrix[0][1] = 2 * (q->x * q->y - q->w * q->z);
    matrix[0][2] = 2 * (q->x * q->z + q->w * q->y);
    matrix[1][0] = 2 * (q->x * q->y + q->w * q->z);
    matrix[1][1] = 2 * (q->w * q->w + q->y * q->y) - 1;
    matrix[1][2] = 2 * (q->y * q->z - q->w * q->x);
    matrix[2][0] = 2 * (q->x * q->z - q->w * q->y);
    matrix[2][1] = 2 * (q->y * q->z + q->w * q->x);
    matrix[2][2] = 2 * (q->w * q->w + q->z * q->z) - 1;
}

void bno055_matrix_vector_multiply(double matrix[3][3], bno055_vector_t *refGravity, bno055_vector_t *newGravity) {
	newGravity->x = matrix[0][0] * refGravity->x + matrix[0][1] * refGravity->y + matrix[0][2] * refGravity->z;
	newGravity->y = matrix[1][0] * refGravity->x + matrix[1][1] * refGravity->y + matrix[1][2] * refGravity->z;
	newGravity->z = matrix[2][0] * refGravity->x + matrix[2][1] * refGravity->y + matrix[2][2] * refGravity->z;

}

void bno055_normalize(bno055_vector_t *q){
	double norm = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

	if (norm < 1e-6) {  // Prevent division by zero
		return;
	}

	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

void bno055_quaternion_to_euler(bno055_vector_t *q, bno055_vector_t *euler) {

	bno055_normalize(q);

	double qw = q->w;
	double qx = q->x;
    double qy= q->y;
    double qz = q->z;

	double sqx = qx * qx;
	double sqy = qy * qy;
	double sqz = qz * qz;

    // Roll
    double roll = atan2(2.0 * (qw * qx + qy * qz), 1 - 2 * (sqx + sqy));
    euler->y = RAD_TO_DEG(roll);

    // Pitch
    double pitch = asin(2.0 * (qw * qy - qx * qz));
    euler->x = RAD_TO_DEG(pitch);

    // Yaw
    double yaw = atan2(2.0 * (qw * qz + qx * qy), 1 - 2 * (sqy + sqz));
    euler->z = fmod((RAD_TO_DEG(yaw) + 360.0), 360.0);

}

bno055_vector_t bno_quaternion  = {0}, bno_accel  = {0}, bno_gyro  = {0};

void bno055_flush() {
	bno055_get_vector_quaternion(&bno_quaternion);
	bno055_normalize(&bno_quaternion);
	bno055_get_vector_accelerometer(&bno_accel);
	bno055_get_vector_gyroscope(&bno_gyro);
}

// Fast calibration-on-boot helper:
//  - Loads offsets from EEPROM
//  - Polls calibration state for a short timeout while logging progress
//  - If fully calibrated, reads and stores the current calibration and displays it
static void bno055_fast_calibration_on_boot(void) {
    bno055_calibration_state_t calState;
    bno055_offsets_t calData;

    LOG_MESSAGE(LOG_INFO, "BNO055: Fast calibration on boot - loading EEPROM offsets");
    bno055_load_calibration_data_sd();

    // Give the sensor a moment to apply offsets and stabilize
    HAL_Delay(50);

    uint32_t start = HAL_GetTick();
    const uint32_t timeout_ms = CAL_TIMEOUT_MS; // 15 seconds for a quick boot calibration

    while (HAL_GetTick() - start < timeout_ms) {
        // Query calibration state
        bno055_get_calibration_state(&calState);

        if (calState.sys == 3 && calState.gyro == 3 && calState.accel == 3 && calState.mag == 3) {
            LOG_MESSAGE(LOG_INFO, "BNO055: Fully calibrated on boot (sys=%d gyro=%d accel=%d mag=%d)", calState.sys, calState.gyro, calState.accel, calState.mag);

            // Read and persist current calibration offsets (will also display them)
            bno055_get_calibration_data(&calData, &calState);

            return;
        }

        // Log intermediate calibration progress at DEBUG level
        LOG_MESSAGE(LOG_DEBUG, "BNO055 calibration progress - sys=%d gyro=%d accel=%d mag=%d", calState.sys, calState.gyro, calState.accel, calState.mag);

        HAL_Delay(500);
    }

    // If we reach here calibration didn't complete within timeout - log final state
    bno055_get_calibration_state(&calState); // final read
    LOG_MESSAGE(LOG_WARN, "BNO055: Fast calibration timed out after %lu ms. Final state sys=%d gyro=%d accel=%d mag=%d", timeout_ms, calState.sys, calState.gyro, calState.accel, calState.mag);
}
