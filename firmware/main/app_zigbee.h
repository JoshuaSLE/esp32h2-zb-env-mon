#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Application Zigbee init.
     *
     * @return esp_err_t
     */
    esp_err_t app_zigbee_init(void);

#ifdef __cplusplus
}
#endif