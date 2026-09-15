#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define DISPLAY_TIMEOUT_MIN_MS (5 * 1000)
#define DISPLAY_TIMEOUT_MAX_MS (60 * 1000)

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Sevent types
     *
     */
    typedef enum sensor_event_type
    {
        SENSOR_EVENT_PRESENCE_DETECTED,
        SENSOR_EVENT_BME280_UPDATED,
    } sensor_event_type_t;

    /**
     * @brief Sensor event data structure
     *
     */
    typedef struct sensor_event
    {
        sensor_event_type_t type;
        float temp;
        float hum;
        float press;
    } sensor_event_t;

    /**
     * @brief Initialize BME280, VCNL4010, GPIO ISR, and start the sensor monitoring task.
     *
     * @param bus_handle Shared I2C master bus handle.
     * @param panel_handle Initialized LCD display handle.
     * @param zb_queue Queue handle to post sensor events to the Zigbee task.
     * @return esp_err_t
     */
    esp_err_t sensor_manager_init(i2c_master_bus_handle_t bus_handle,
                                  esp_lcd_panel_handle_t panel_handle,
                                  QueueHandle_t zb_queue);

    /**
     * @brief Update the display auto-off timeout dynamically.
     *
     * @param timeout_ms Target timeout in milliseconds (5000 - 60000 ms).
     * @return esp_err_t
     */
    esp_err_t sensor_manager_set_display_timeout_ms(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif