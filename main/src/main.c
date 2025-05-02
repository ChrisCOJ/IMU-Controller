#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../include/ble_conn_setup.h"
#include "../include/mpu_i2c.h"
#include "../include/mc_general_defs.h"
#include "../include/mc_hid_defs.h"


#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "sdkconfig.h"

#define ACCEL_2G_RATIO              16384               // Accelerometer values are divided by this ratio and converted to Gs (Sensitivity = 2G)
#define GYRO_250_DEG_RATIO          131.0               // Gyroscope values are divided by this ratio and converted to degrees (Sensitivity = 250 deg)
#define SENS_RATIO                  0.5                 // Divisor to calculate final pointer sensitivity
#define LEFT_CLICK_GPIO             GPIO_NUM_23
#define CALIBRATION_DONE            0
#define CALIBRATION_IN_PROGRESS     1
#define AVERAGE_Y_SAMPLE_SIZE       20


// Initialize buffers for acceleration and gyro data
static uint8_t x, y;
static int16_t accel_arr[3];
static size_t accel_arr_size = sizeof(accel_arr) / sizeof(accel_arr[0]);
static int16_t gyro_arr[3];
static size_t gyro_arr_size = sizeof(gyro_arr) / sizeof(gyro_arr[0]);

float averaged_y_accel_arr[AVERAGE_Y_SAMPLE_SIZE];
uint8_t calibration_roll_reading_idx = 0;       // Array index used to initially populate the averaged_y_accel_arr
uint8_t roll_reading_idx = 0;                   // Array index used to update the averaged_y_accel_arr and get average value


int calibrate_remote(float accel_y) {
    /* Calculate MPU roll for more responsive movement
    * 1. Average y-axis accelerometer readings (e.g. over 20 readings).
    * 2. Determine a gyroscope y-axis deadzone to filter out random noise (e.g. roll will be read, only if
    *    gyroscope y-axis reading surpasses 10/-10).
    * 3. Ignore large y-axis accelerometer values exceeding the maximum gravitational acceleration.
    * 4. Following this preprocessing, the y-axis acclerometer reading will now serve as an estimate for roll.
    */
    if (calibration_roll_reading_idx >= AVERAGE_Y_SAMPLE_SIZE) {
        return CALIBRATION_DONE;
    }
    averaged_y_accel_arr[calibration_roll_reading_idx] = accel_y;
    ++calibration_roll_reading_idx;

    return CALIBRATION_IN_PROGRESS;
}


float get_average_roll(float accel_y) {
    // * Returns average roll ranging from 0.0 to 2.0

    float sum = 0;
    float average = 0;

    for (int i=0; i < AVERAGE_Y_SAMPLE_SIZE; ++i) {
        sum += averaged_y_accel_arr[i];
    }
    average = sum / AVERAGE_Y_SAMPLE_SIZE;

    /* Once an average has been calulated, replace the oldest reading in the average roll buffer with a new reading,
    Effectively creating a sliding window */
    ++roll_reading_idx;
    if (roll_reading_idx >= AVERAGE_Y_SAMPLE_SIZE) {
        roll_reading_idx = 0;
    }

    averaged_y_accel_arr[roll_reading_idx] = accel_y;

    return average;
}


int process_mpu_output_to_hid_report(i2c_master_dev_handle_t dev_handle, uint8_t* hid_report, size_t hid_report_size) {
    int status;
    // char *axes[3] = { "X", "Y", "Z" };  // Used for logging

    // Read raw acceleration values from the MPU
    status = mpu_read_data(MPU_ACCEL_DATA, dev_handle, accel_arr, accel_arr_size);
    if (status != MPU_READ_SUCCESS) {
        ESP_LOGE(MAIN_TAG, "Failed to read acceleration values from the mpu into the provided buffer");
        return -1;
    }
    // Read raw gyro values from the MPU
    status = mpu_read_data(MPU_GYRO_DATA, dev_handle, gyro_arr, gyro_arr_size);
    if (status != MPU_READ_SUCCESS) {
        ESP_LOGE(MAIN_TAG, "Failed to read gyro values from the mpu into the provided buffer");
        return -1;
    }

    // Process MPU movement and rotation data into cursor xy position

    // Convert acceleration range to Gs (max sens = 2G / second)
    float accel_y = (float)accel_arr[1] / (float)ACCEL_2G_RATIO;

    // Convert gyroscope range to degrees (max sens = 250 deg / second)
    // gyro[0] = x, gyro[1] = y, gyro[2] = z
    for (int i=0; i < gyro_arr_size; ++i) {
        gyro_arr[i] /= GYRO_250_DEG_RATIO;
        // ESP_LOGI("Gyroscope", "%s = %d", axes[i], gyro_arr[i]);
    }

    if (calibrate_remote(accel_y) == CALIBRATION_DONE) {
        float roll = get_average_roll(accel_y);

        x = (uint8_t)((gyro_arr[1] + gyro_arr[2] * roll) * SENS_RATIO);
        y = -(uint8_t)((gyro_arr[2] + gyro_arr[1] * roll) * SENS_RATIO);

        // Update HID report's xy values
        if (!gpio_get_level(LEFT_CLICK_GPIO)) {     // Checks if the left click button is pressed
            hid_report[0] = 1;                      // 1 = HID mouse left click pressed
            x = 0, y = 0;
        } else {
            hid_report[0] = 0;
        }

        hid_report[1] = y;
        hid_report[2] = x;
    }

    return 0;
}


void send_hid_report(void* param) {
    i2c_master_dev_handle_t i2c_dev_handle = (i2c_master_dev_handle_t)param;

    while(1) {
        if (report_enabled_notifications) {
            int status = process_mpu_output_to_hid_report(i2c_dev_handle, hid_report, sizeof(hid_report));
            if (status) {
                ESP_LOGE(MAIN_TAG, "Failed to update hid report!");
            }

            // Send processed report notification to the BLE host
            esp_err_t ret = esp_ble_gatts_send_indicate(motion_controller_profile_tab[HID_PROFILE_APP_IDX].gatts_if, 
                                                        motion_controller_profile_tab[HID_PROFILE_APP_IDX].conn_id, 
                                                        hid_handle_table[IDX_CHAR_HID_REPORT_VAL],
                                                        HID_REPORT_LEN, hid_report, false);
            if (ret != ESP_OK) {
                ESP_LOGE(GATTS_TAG, "Failed to send report! Error code %d", ret);
            }
            // ESP_LOGI(GATTS_TAG, "Sent report successfully!");
        }

        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


void app_main(void) {

    // ********************* Setup ********************* !
    static i2c_master_bus_handle_t mst_bus_handle;
    static i2c_master_dev_handle_t mpu_dev_handle;

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

    i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,  // Sets address length of the slave device
        .device_address = MPU6050_I2C_ADDR,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &mpu_cfg, &mpu_dev_handle));

    mpu_init(mpu_dev_handle);  // Wake up mpu sensor and configure it's registers

    // Configure the left click button GPIO pin
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,         // Disable interrupt
        .mode         = GPIO_MODE_INPUT,           // Set GPIO as input    
        .pull_down_en = GPIO_PULLDOWN_DISABLE,     // Disable pulldown resistors
        .pull_up_en   = GPIO_PULLUP_ENABLE,        // Enable pullup resistors
        .pin_bit_mask = 1 << LEFT_CLICK_GPIO
    };

    gpio_config(&io_conf);
    // *************************************************** !

    xTaskCreate(send_hid_report, "MC", 4096, mpu_dev_handle, 5, NULL);

    // Release i2c resources
    // i2c_master_bus_rm_device(&dev_handle);
    // i2c_del_master_bus(&mst_bus_handle);
}