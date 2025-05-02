#ifndef mpu_i2c_h
#define mpu_i2c_h

#include "driver/gpio.h"
#include "driver/i2c_master.h"


#define I2C_MASTER_PORT                 I2C_NUM_0     // Port number used by esp32
#define I2C_MASTER_GPIO_SDA             18            // GPIO number for serial data bus (SDA)
#define I2C_MASTER_GPIO_SCL             19            // GPIO number for serial clock bus (SCL)
#define I2C_SCL_CLK_HZ                  100000        // I2C Clock speed in hz
#define I2C_MASTER_TIMEOUT_MS           1000          // Time in ms to timeout communication

#define MPU6050_I2C_ADDR                0x68          // I2C address of the MPU6050 sensor
#define MPU6050_PWR_MGMT1_REG           0x6B  
#define MPU6050_WAKE_UP_SIG             0x00        
#define MPU6050_RESET_BIT               7
#define MPU6050_SLEEP_BIT               6

#define MPU6050_ACCEL_REG               0x3B          // The address of the first MPU6050's accelerometer register
#define MPU6050_ACCEL_CONFIG_REG        0x1C          // Accel Config register address. Bits 4 and 3 are used to set the maximum value of the accelerometer outputs
#define MPU6050_ACCEL_2G                0x00          // Value to configure the accelerometer sensitivity to be 2g
#define MPU6050_ACCEL_4G                0x08          // Value to configure the accelerometer sensitivity to be 4g
#define MPU6050_ACCEL_8G                0x10          // Value to configure the accelerometer sensitivity to be 8g

#define MPU6050_GYRO_REG                0x43          // The address of the first MPU6050's gyro register
#define MPU6050_GYRO_CONFIG_REG         0x1B          // Gyro Config register address. Bits 4 and 3 are used to set the maximum value of the gyro outputs
#define MPU6050_GYRO_250_DEG            0x00          // Value to configure the gyro sensitivity to be 250 degrees / second
#define MPU6050_GYRO_500_DEG            0x08          // Value to configure the gyro sensitivity to be 500 degrees / second
#define MPU6050_GYRO_1000_DEG           0x10          // Value to configure the gyro sensitivity to be 1000 degrees / second
#define MPU6050_GYRO_2000_DEG           0x18          // Value to configure the gyro sensitivity to be 2000 degrees / second


#define MPU_READ_SUCCESS                0             // Success status code for reading data from the MPU

#define MPU_ACCEL_DATA                  0             // Acceleration data type parameter used in mpu_read_data() function
#define MPU_GYRO_DATA                   1             // Gyro data type parameter used in mpu_read_data() function


esp_err_t mpu_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t *read_buffer, size_t read_buffer_size);

esp_err_t mpu_reg_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_address, uint8_t data);

void mpu_init(i2c_master_dev_handle_t dev_handle);

int mpu_read_data(int data_type, i2c_master_dev_handle_t dev_handle, int16_t *reader_array, size_t reader_array_size);

#endif
