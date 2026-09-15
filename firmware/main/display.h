#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Display init.
     *
     * @param bus_handle I2C bus handle.
     * @param panel_handle Display panel handle.
     * @return esp_err_t
     */
    esp_err_t display_init(i2c_master_bus_handle_t bus_handle, esp_lcd_panel_handle_t *panel_handle);

    /**
     * @brief Display deinit.
     *
     * @param panel_handle Display panel handle.
     * @return esp_err_t
     */
    esp_err_t display_deinit(esp_lcd_panel_handle_t *panel_handle);

    /**
     * @brief Display show readings.
     *
     * @param panel_handle Display panel handle.
     * @param pressure Pressure (Pa).
     * @param temp Temperature (C).
     * @param humidity Humidity (%).
     * @return esp_err_t
     */
    esp_err_t display_show_readings(esp_lcd_panel_handle_t panel_handle, float pressure, float humidity, float temp);

    /**
     * @brief Display off on.
     *
     * @param panel_handle Display panel handle.
     * @param off_on 0 display off, 1 display on.
     * @return esp_err_t
     */
    esp_err_t display_off_on(esp_lcd_panel_handle_t panel_handle, bool off_on);

    /**
     * @brief Display off.
     *
     * @param panel_handle Display panel handle.
     * @return esp_err_t
     */
    static inline esp_err_t display_off(esp_lcd_panel_handle_t panel_handle)
    {
        return display_off_on(panel_handle, false);
    }

    /**
     * @brief Display on.
     *
     * @param panel_handle Display panel handle.
     * @return esp_err_t
     */
    static inline esp_err_t display_on(esp_lcd_panel_handle_t panel_handle)
    {
        return display_off_on(panel_handle, true);
    }

#ifdef __cplusplus
}
#endif