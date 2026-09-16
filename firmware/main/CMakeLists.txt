idf_component_register(SRCS "alarm_timer.c" "app_zigbee.c" "sensor_manager.c" "display.c" "i2c_bus.c" "app_main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2c esp_lcd esp_driver_gpio esp_timer nvs_flash esp_timer
                        bme280 vcnl4010)
