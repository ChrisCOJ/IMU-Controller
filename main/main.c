#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
// #include "soc/gpio_struct.h"
// #include "hal/gpio_ll.h"


#define I2C_MASTER_PORT                 I2C_NUM_0     // Port number used by esp32
#define I2C_MASTER_GPIO_SDA             25            // GPIO number for serial data bus (SDA)
#define I2C_MASTER_GPIO_SCL             26            // GPIO number for serial clock bus (SCL)
#define I2C_SCL_CLK_HZ                  100000        // I2C Clock speed in hz
#define I2C_MASTER_TIMEOUT_MS           1000          // Time in ms to timeout communication

#define MPU6050_I2C_ADDR                0x68          // I2C address of the MPU6050 sensor

#define MPU6050_PWR_MGMT_REG            0x6B          
#define MPU6050_RESET_SIG               0x80
#define MPU6050_SLEEP_SIG               0x40

#define MPU6050_ACCEL_REG               0x3B          // The address of the first MPU6050's accelerometer register
#define MPU6050_ACCEL_CONFIG_REG        0x1c          // Bits 4 and 3 are used to set the maximum value of the accelerometer outputs
#define MPU6050_ACCEL_2G                0x00          // Value to configure the maximum value of the accelerometer to be 2g
#define MPU6050_ACCEL_4G                0x04          // Value to configure the maximum value of the accelerometer to be 4g
#define MPU6050_ACCEL_8G                0x08          // Value to configure the maximum value of the accelerometer to be 8g


i2c_master_bus_handle_t mst_bus_handle;
i2c_master_dev_handle_t dev_handle;
static const char *TAG = "I2C";

static esp_err_t mpu_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t *read_buffer, size_t read_buffer_size) {

    return i2c_master_transmit_receive(dev_handle, &reg_address, 1, read_buffer, read_buffer_size, 
                                       I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static esp_err_t mpu_reg_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t data) {

    uint8_t write_buf[2] = { reg_address, data };
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


void master_init(int slave_addr) {
    // Configure the i2c bus
    i2c_master_bus_config_t mst_cfg = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_GPIO_SDA,
        .scl_io_num = I2C_MASTER_GPIO_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Initialize the configured i2c bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&mst_cfg, &mst_bus_handle));

    // Configure the i2c master device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,  // Address bit length of the slave device
        .device_address = slave_addr,
        .scl_speed_hz = I2C_SCL_CLK_HZ,
    };

    // Initialize the configured i2c master device
    ESP_ERROR_CHECK(i2c_master_bus_add_device(mst_bus_handle, &dev_cfg, &dev_handle));
}


void mpu_init() {
    // Reset the MPU6050 registers to their default values
    mpu_reg_write_byte(dev_handle, MPU6050_PWR_MGMT_REG, MPU6050_RESET_SIG);

    // Configure the MPU6050 accelerometer 
    mpu_reg_write_byte(dev_handle, MPU6050_ACCEL_CONFIG_REG, MPU6050_ACCEL_8G);
}


void readAccel(){
    /**  Function that reads acceleration data from an MPU6050 over I2C  **/

    // Write the address of the fisrt accelerometer address and read the next 6 bytes into accel_data
    uint8_t accel_data[6];
    esp_err_t err = mpu_read_reg(dev_handle, MPU6050_ACCEL_REG, accel_data, sizeof(accel_data));

    if (err == ESP_OK) {
        int16_t accel_x = (accel_data[0] << 8) | accel_data[1];
        int16_t accel_y = (accel_data[2] << 8) | accel_data[3];
        int16_t accel_z = (accel_data[4] << 8) | accel_data[5];

        ESP_LOGI(TAG, "Acceleration X: %d Y: %d Z: %d", accel_x, accel_y, accel_z);
    }
}


void app_main(void)
{
    // Setup
    master_init(MPU6050_I2C_ADDR);
    mpu_init();

    // Read the value of the mpu6050's accel config register
    uint8_t accel_config_read[1];
    esp_err_t err = mpu_read_reg(dev_handle, MPU6050_ACCEL_CONFIG_REG, accel_config_read, sizeof(accel_config_read));

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Value of accel config register: %u", accel_config_read[0]);
    }

    // // Release i2c resources
    // i2c_master_bus_rm_device(&dev_handle);
    // i2c_del_master_bus(&mst_bus_handle);

    // Loop
    while(1) {
        readAccel();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}