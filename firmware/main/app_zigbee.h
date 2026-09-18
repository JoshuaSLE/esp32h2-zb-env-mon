#pragma once

#include "esp_err.h"

#define APP_ZB_ENV_MONITOR_DEVICE_ID 0xFF02
#define APP_ZB_CUSTOM_CLUSTER_ID 0xFF01
#define APP_ZB_DISPLAY_TIMEOUT_ATTR_ID 0x0000

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