idf_component_register(SRCS "src/ble_conn_setup.c" "src/mpu_i2c.c" "src/main.c"
                    INCLUDE_DIRS "include"
                    REQUIRES nvs_flash bt driver)