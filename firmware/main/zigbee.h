#pragma once

#include "bme280.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Initialize Zigbee stack, register signal handler, create data model.
     *
     * @return esp_err_t
     */
    esp_err_t zigbee_init(void);

    /**
     * @brief Report the BME280 readings.
     * 
     * @param reading BME280 readings data.
     */
    void zigbee_report_bme280(const bme280_data_t *reading);

#ifdef __cplusplus
}
#endif