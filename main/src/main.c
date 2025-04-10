#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/ble_conn_setup.h"
#include "../include/mpu_i2c.h"
#include "../include/mc_general_defs.h"

#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"


void app_main(void)
{

    // ********************* Setup ********************* !
    // Initialize bluetooth
    if (bt_init() != ESP_OK){
        ESP_LOGE(GATTS_TAG, "%s Failed to initialize bluetooth", __func__);
        return;
    }

    // Config and init the i2c bus and mpu6050 device
    i2c_master_bus_config_t mst_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_GPIO_SDA,
        .scl_io_num = I2C_MASTER_GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&mst_cfg, &mst_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,  // Sets address length of the slave device
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &dev_cfg, &dev_handle));

    mpu_init();  // Wake up mpu sensor and configure it's registers
    // *************************************************** !

    /** Gyro and Accelerometer values are divided by this ratio and converted to
    mouse sensitivity (pixels per second) **/
    uint16_t mouse_sens_ratio = 1000;

    // This array will hold the raw 16-bit acceleration values for the X, Y, and Z axes.
    int16_t raw_accel_arr[3];
    size_t raw_accel_arr_size = sizeof(raw_accel_arr) / sizeof(raw_accel_arr[0]);
    char *axes[3] = { "X", "Y", "Z" };
    // Loop
    while(1) {
        // Reads the MPU6050's raw acceleration values into raw_accel_arr
        mpu_read_accel(raw_accel_arr, raw_accel_arr_size);

        // Normalize raw acceleration values(signed int16) to pixels per second values based on mouse_sens_ratio and log them.
        for (int i=0; i < raw_accel_arr_size; ++i) {
            raw_accel_arr[i] /= mouse_sens_ratio;
            //ESP_LOGI(MPU_TAG, "%s: %d", axes[i], raw_accel_arr[i]);
        }
        //ESP_LOGI(MPU_TAG, "");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Release i2c resources
    // i2c_master_bus_rm_device(&dev_handle);
    // i2c_del_master_bus(&mst_bus_handle);
}