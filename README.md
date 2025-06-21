🖱️ **AirNav — A BLE Air Mouse Powered by ESP32 and MPU6050**

AirNav turns an ESP32 and an MPU6050 IMU into a fully functional Bluetooth Low Energy (BLE) HID mouse by detecting hand motion and translating it into cursor movement and clicks.

🚀 Features

- BLE HID mouse profile with full GATT service and report descriptors
- MPU6050-based motion tracking — detects gyroscope and accelerometer movement
- Motion calibration and filtering for stable cursor control
- Hardware left-click support via GPIO
- Built with ESP-IDF using low-level control of I2C, BLE GATT, and HID layers

🧰 Hardware Requirements

- ESP32 development board
- MPU6050 IMU module (connected via I2C)
- Tactile button (for left-click), connected to GPIO 23 by default
- Power source (e.g., USB or battery)
