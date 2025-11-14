/*
 * MS4525DO.c
 *
 *  Created on: Oct 26, 2024
 *      Author: chant, ella
 */
#include "ms4525do.h"

struct MS4525DO_t ms4525do;
/**
 * Configures which i2c port MS4525DO is on
 */
void ms4525d0_init(I2C_HandleTypeDef *hi2c) {
	/*Set i2c handle*/
	ms4525do.i2c_handle = hi2c;
	/*Initialize everything to defaults*/
	SensorStatus initStatus = normal;
	ms4525do.sensor_status = initStatus;
	ms4525do.raw_data.pressure = 0;
	ms4525do.raw_data.temperature = 0;
	ms4525do.processed_data.pressure_psi = 0;
	ms4525do.processed_data.temperature_C = 0;
	ms4525do.processed_data.airspeed_mps = 0;}

void read_ms4525do(void) {
	uint8_t data_buffer[4]; //data buffer to store raw I2C data
	HAL_StatusTypeDef i2c_status = HAL_I2C_Master_Receive(ms4525do.i2c_handle, ADDRESS_I2C_MS4525DO << 1, data_buffer, sizeof(data_buffer), HAL_MAX_DELAY);
#ifdef VERBOSE_MODE_EN
    if (i2c_status == HAL_OK) {
        printf("HAL_OK\r\n");
    } else if (i2c_status == HAL_ERROR) {
    	printf("HAL_ERROR\r\n");
    } else if (i2c_status == HAL_BUSY) {
    	printf("HAL_BUSY\r\n");
    } else if (i2c_status == HAL_TIMEOUT) {
        printf("HAL_TIMEOUT\r\n");
    }
    //diagnose HAL error
    uint32_t i2c_error = HAL_I2C_GetError(ms4525do.i2c_handle);
    if (i2c_error == HAL_I2C_ERROR_NONE) {
      printf("no errors \r\n");
    } else if (i2c_error == HAL_I2C_ERROR_BERR) {
      printf("HAL_I2C_ERROR_BERR\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_ARLO) {
      printf("HAL_I2C_ERROR_ARLO\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_AF) {
      printf("HAL_I2C_ERROR_AF\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_OVR) {
      printf("HAL_I2C_ERROR_OVR\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_DMA) {
      printf("HAL_I2C_ERROR_DMA\r\n");
    } else if (i2c_error == HAL_I2C_ERROR_TIMEOUT) {
      printf("HAL_I2C_ERROR_TIMEOUT\r\n");
    }
    //print the raw bytes
    printf("%u, ",data_buffer[0]);
    printf("%u, ",data_buffer[1]);
    printf("%u, ",data_buffer[2]);
    printf("%u \r\n",data_buffer[3]);
#endif
    /*Status - 2 Bits*/
    uint8_t read_status = (uint8_t)(data_buffer[0] >> 6);
    SensorStatus stat;
    switch (read_status){
    case 0:
    	stat = normal;
    	break;
    case 1:
    	stat = reserved;
    	break;
    case 2:
    	stat = stale;
    	break;
    case 3:
    	stat = fault;
    	break;
    default:
    	stat = unknown;
    	break;
    }
    ms4525do.sensor_status = stat;
    /*Pressure - 14 Bits*/
    ms4525do.raw_data.pressure = (((uint16_t)data_buffer[0] << 8) & 0x3F00) + ((uint16_t)data_buffer[1] << 0); 	//Combines High and Low Pressure. Clears status bits
    /*Temperature - 11 Bits*/
    ms4525do.raw_data.temperature = ((uint16_t)data_buffer[2] << 3) + ((uint16_t)data_buffer[3] >> 5);			//Combines High and Low Temperature. Clears last 5 bits.

    if(TYPE_MS4525DO) {
    	 /*Type A*/
    	 ms4525do.processed_data.pressure_psi = ((((double)ms4525do.raw_data.pressure-1638.3)*(PMAX_PSI_MS4525DO - PMIN_PSI_MS4525DO ))/13106.4)+PMIN_PSI_MS4525DO;
    } else {
    	/*Type B*/
    	ms4525do.processed_data.pressure_psi = ((((double)ms4525do.raw_data.pressure-819.15)*(PMAX_PSI_MS4525DO-PMIN_PSI_MS4525DO))/14744.7)+PMIN_PSI_MS4525DO;
    }
    ms4525do.processed_data.temperature_C = (((double)ms4525do.raw_data.temperature*200.0)/2047.0)-50.0;

    /*Output swings positive when Port 1> Port 2, negative vice versa. Output is 50% (8192D) when Port 1 = Port 2*/
    if(ms4525do.processed_data.pressure_psi >= 0) {
    	//Positive to denote Port 1 > Port 2
    	ms4525do.processed_data.airspeed_mps = sqrt((2*6894.7*ms4525do.processed_data.pressure_psi)/AIR_DENSITY);
    } else {
        //Negative to denote Port 1 < Port 2
    	ms4525do.processed_data.airspeed_mps = -sqrt((2*6894.7*abs(ms4525do.processed_data.pressure_psi))/AIR_DENSITY);
    }

#ifdef WIND_TUNNEL_EN
    printf("%u", ms4525do.raw_data.pressure);
    printf(", %f", ms4525do.processed_data.pressure_psi);
    printf(", %f", ms4525do.processed_data.airspeed_mps);
#endif
}


