#include "display.h"
#include "i2c_bus.h"
#include "sensor_manager.h"
#include "zigbee.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "app_main";

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static esp_err_t app_power_save_init(void)
{
#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true,
#endif
    };
    return esp_pm_configure(&pm_config);
#else
    return ESP_OK;
#endif
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing system...");
    ESP_ERROR_CHECK(app_power_save_init());

    ESP_LOGI(TAG, "Initializing system peripherals...");
    ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus_handle));
    ESP_ERROR_CHECK(display_init(i2c_bus_handle, &panel_handle));

    ESP_ERROR_CHECK(sensor_manager_init(i2c_bus_handle, panel_handle));

    ESP_ERROR_CHECK(zigbee_init());

    ESP_LOGI(TAG, "System ready");
}