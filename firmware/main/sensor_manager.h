#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        SENSOR_EVENT_PRESENCE_DETECTED,
        SENSOR_EVENT_BME280_UPDATED,
    } sensor_event_type_t;

    typedef struct
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
    esp_err_t presence_init(i2c_master_bus_handle_t bus_handle,
                            esp_lcd_panel_handle_t panel_handle,
                            QueueHandle_t zb_queue);

#ifdef __cplusplus
}
#endif