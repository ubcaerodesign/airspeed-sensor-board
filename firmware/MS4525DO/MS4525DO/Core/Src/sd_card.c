#include "sd_card.h"
#include <string.h>
#include <stdio.h>
#include "sensors.h"

FRESULT result; /* FatFs function common result code */
UINT byteswritten; /* File write count */
static FRESULT mount_status = FR_NO_FILESYSTEM;
FIL SDFile;
FATFS SDFatFS;

static uint8_t mount_sd_card(void);

void save_sensors_sd(sensor_data_t *sensor_data) {

    char csv_line[256];
    char header[] =
        "acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z,q0,q1,q2,q3,time\n";

    snprintf(csv_line, sizeof(csv_line),
             "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
             "%.6f,%.6f,%.6f,%.6f,%.2f\n",
             sensor_data->accel_x,
             sensor_data->accel_y,
             sensor_data->accel_z,
             sensor_data->gyro_x,
             sensor_data->gyro_y,
             sensor_data->gyro_z,
             sensor_data->q0,
             sensor_data->q1,
             sensor_data->q2,
             sensor_data->q3,
             sensor_data->time);

    if (mount_sd_card() != FR_OK) return;

    UINT byteswritten;

    // CREATE FILE IF MISSING + APPEND
    FRESULT res = f_open(&SDFile,
                         "sensor_data.csv",
                         FA_WRITE | FA_OPEN_ALWAYS);
    if (res != FR_OK) return;

    // move to end for append
    f_lseek(&SDFile, f_size(&SDFile));

    // write header if file is empty
    if (f_tell(&SDFile) == 0) {
        f_write(&SDFile, header, strlen(header), &byteswritten);
    }

    // write data
    f_write(&SDFile, csv_line, strlen(csv_line), &byteswritten);

    // force flush
    f_sync(&SDFile);

    f_close(&SDFile);
}

FRESULT sd_log_line_to_file(const char* filename, const char* log_line) {
    FRESULT res;

    if (mount_sd_card() != FR_OK) return FR_NOT_READY;

    // Open file for read/write
    res = f_open(&SDFile, filename, FA_WRITE | FA_READ);
    if (res != FR_OK) return res;

    // Move to end of file
    f_lseek(&SDFile, f_size(&SDFile));

    // Write line
    res = f_write(&SDFile, log_line, strlen(log_line), &byteswritten);
    f_close(&SDFile);

    return res;
}

FRESULT sd_append(const char *filename, const char *line) {
    FRESULT res;

    // Ensure SD card is mounted
    res = f_mount(&SDFatFS, "", 0);
    if (res != FR_OK) return res;

    // Open file for read/write (create if it doesn't exist)
    res = f_open(&SDFile, filename, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) return res;

    // Move to end of file to append
    res = f_lseek(&SDFile, f_size(&SDFile));
    if (res != FR_OK) {
        f_close(&SDFile);
        return res;
    }

    // Write the line
    res = f_write(&SDFile, line, strlen(line), &byteswritten);

    // Close the file
    f_close(&SDFile);

    return res;
}

static uint8_t mount_sd_card(void){
    if (mount_status != FR_OK){
        mount_status = f_mount(&SDFatFS, "", 0);
    }
    return mount_status;
}
