#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/mpu_i2c.h"

#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"


/********************************************
 * This file contains mpu6050 functionality 
 *******************************************/

i2c_master_bus_handle_t mst_bus_handle;
i2c_master_dev_handle_t dev_handle;

void mpu_init() {
    // Wake up the MPU6050
    mpu_reg_write_byte(dev_handle, MPU6050_PWR_MGMT1_REG, MPU6050_WAKE_UP_SIG);

    // Configure the MPU6050 accelerometer 
    mpu_reg_write_byte(dev_handle, MPU6050_ACCEL_CONFIG_REG, MPU6050_ACCEL_8G);
}


esp_err_t mpu_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t *read_buffer, size_t read_buffer_size) {
    // Key takeaway: Reads data from reg_address into read_buffer
    // First writes the address of the register to be read from, then reads data from that register into a user defined buffer

    return i2c_master_transmit_receive(dev_handle, &reg_address, 1, read_buffer, read_buffer_size, 
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


esp_err_t mpu_reg_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t data) {
    // Sends the address of the register to be written to, then sends the actual data to be written.

    uint8_t write_buf[2] = { reg_address, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


int mpu_read_accel(int16_t *reader_array, size_t reader_array_size) {
    /**  Function that reads acceleration data from an MPU6050 over I2C  **/
    if (sizeof(reader_array_size) < 3) {  // Error handling, reader_arrat size must be >= 3
        ESP_LOGE("mpu_read_accel()", "Invalid reader_array parameter size. Size must be >= 3");
        return -1;
    }

    // Write the address of the fisrt accelerometer address and read the next 6 bytes into accel_data
    uint8_t accel_data[6];
    esp_err_t err = mpu_read_reg(dev_handle, MPU6050_ACCEL_REG, accel_data, sizeof(accel_data));

    if (err == ESP_OK) {
        int16_t accel_x = ((accel_data[0] << 8) | accel_data[1]);
        reader_array[0] = accel_x;
        int16_t accel_y = ((accel_data[2] << 8) | accel_data[3]);
        reader_array[1] = accel_y;
        int16_t accel_z = ((accel_data[4] << 8) | accel_data[5]);
        reader_array[2] = accel_z;

        // ESP_LOGI(TAG, "Acceleration X: %d Y: %d Z: %d", accel_x, accel_y, accel_z);
    }

    return 0;
}