#include "display.h"
#include "i2c_bus.h"
#include "sensor_manager.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "app_main";

static QueueHandle_t zigbee_event_queue = NULL;

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing system peripherals...");

    zigbee_event_queue = xQueueCreate(10, sizeof(sensor_event_t));
    ESP_ERROR_CHECK(zigbee_event_queue ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(i2c_bus_init(&i2c_bus_handle));
    ESP_ERROR_CHECK(display_init(i2c_bus_handle, &panel_handle));

    ESP_ERROR_CHECK(sensor_manager_init(i2c_bus_handle, panel_handle, zigbee_event_queue));

    // Zigbee init here
}