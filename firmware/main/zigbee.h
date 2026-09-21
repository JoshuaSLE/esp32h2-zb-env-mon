#pragma once

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

#ifdef __cplusplus
}
#endif