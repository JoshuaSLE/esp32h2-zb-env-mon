#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize BME280, VCNL4010, GPIO ISR, and start the sensor monitoring tasks.
     *
     * @param bus_handle Shared I2C master bus handle.
     * @param panel_handle Initialized LCD display handle.
     * @return esp_err_t
     */
    /* ---- FIX: removed unused zb_queue parameter ---- */
    esp_err_t sensor_manager_init(i2c_master_bus_handle_t bus_handle,
                                  esp_lcd_panel_handle_t panel_handle);

#ifdef __cplusplus
}
#endif