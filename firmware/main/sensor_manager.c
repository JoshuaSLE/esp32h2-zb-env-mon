#include "sensor_manager.h"
#include "bme280.h"
#include "display.h"
#include "vcnl4010.h"

#include "driver/gpio.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define BME280_SAMPLE_INTERVAL_US (60ULL * 1000 * 1000)

static const char *TAG = "sensor_manager";

static QueueHandle_t zigbee_queue = NULL;
static TaskHandle_t presence_task_handle = NULL;

static bme280_handle_t bme280_handle = NULL;
static vcnl4010_handle_t vcnl4010_handle = NULL;
static esp_lcd_panel_handle_t display_handle = NULL;

static void IRAM_ATTR vcnl4010_isr_handler(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(presence_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

static void post_zigbee_event(const sensor_event_t *evt)
{
    if (zigbee_queue == NULL)
    {
        return;
    }
    xQueueSend(zigbee_queue, evt, 0);
}

static int64_t handle_presence_event(const bme280_data_t *cached_reading, bool *display_is_on, int64_t now_us)
{
    uint8_t int_status = 0;
    esp_err_t err = vcnl4010_clear_interrupt(vcnl4010_handle, &int_status);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "failed to clear vcnl4010 interrupt: %s", esp_err_to_name(err));
        return now_us;
    }

    if (!*display_is_on)
    {
        err = display_off_on(display_handle, true);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "failed to turn on display: %s", esp_err_to_name(err));
        }
        *display_is_on = true;
    }

    err = display_show_readings(display_handle, cached_reading->temp, cached_reading->hum, cached_reading->press);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "failed to update display: %s", esp_err_to_name(err));
    }

    post_zigbee_event(&(sensor_event_t){.type = SENSOR_EVENT_PRESENCE_DETECTED});

    return now_us + ((int64_t)CONFIG_APP_DISPLAY_TIMEOUT_MS * 1000);
}

static bool maybe_sample_bme280(bme280_data_t *cached_reading, int64_t *last_bme_read_us,
                                bool display_is_on, int64_t now_us)
{
    if ((now_us - *last_bme_read_us) < BME280_SAMPLE_INTERVAL_US)
    {
        return false;
    }

    esp_err_t err = bme280_trigger_measurement(bme280_handle);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "bme280 trigger failed: %s", esp_err_to_name(err));
        return false;
    }

    bme280_data_t reading;
    err = bme280_read_data(bme280_handle, &reading);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "bme280 read failed: %s", esp_err_to_name(err));
        return false;
    }

    *cached_reading = reading;
    *last_bme_read_us = now_us;

    if (display_is_on)
    {
        err = display_show_readings(display_handle, reading.temp, reading.hum, reading.press);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "failed to update display: %s", esp_err_to_name(err));
        }
    }

    post_zigbee_event(&(sensor_event_t){
        .type = SENSOR_EVENT_BME280_UPDATED,
        .temp = reading.temp,
        .hum = reading.hum,
        .press = reading.press,
    });

    return true;
}

static void maybe_timeout_display(bool *display_is_on, int64_t display_off_target_us, int64_t now_us)
{
    if (!*display_is_on || now_us < display_off_target_us)
    {
        return;
    }

    esp_err_t err = display_off_on(display_handle, false);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "failed to turn off display: %s", esp_err_to_name(err));
        return;
    }
    *display_is_on = false;
}

static void presence_task(void *pvParameters)
{
    bme280_data_t cached_reading = {0};
    int64_t last_bme_read_us = 0;
    int64_t display_off_target_us = 0;

    bool display_is_on = true;

    if (bme280_trigger_measurement(bme280_handle) == ESP_OK)
    {
        (void)bme280_read_data(bme280_handle, &cached_reading);
        last_bme_read_us = esp_timer_get_time();

        display_off_target_us = last_bme_read_us + ((int64_t)CONFIG_APP_DISPLAY_TIMEOUT_MS * 1000);
        (void)display_show_readings(display_handle, cached_reading.temp, cached_reading.hum, cached_reading.press);
    }
    else
    {
        display_off_target_us = esp_timer_get_time() + ((int64_t)CONFIG_APP_DISPLAY_TIMEOUT_MS * 1000);
    }

    while (1)
    {
        uint32_t presence_event = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        int64_t now_us = esp_timer_get_time();

        if (presence_event > 0)
        {
            display_off_target_us = handle_presence_event(&cached_reading, &display_is_on, now_us);
        }

        maybe_sample_bme280(&cached_reading, &last_bme_read_us, display_is_on, now_us);
        maybe_timeout_display(&display_is_on, display_off_target_us, now_us);
    }
}

esp_err_t presence_init(i2c_master_bus_handle_t bus_handle, esp_lcd_panel_handle_t panel_handle, QueueHandle_t zb_queue)
{
    if (!bus_handle || !panel_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    display_handle = panel_handle;
    zigbee_queue = zb_queue;

    bme280_config_t bme_cfg = {
        .bus_handle = bus_handle,
        .i2c_address = CONFIG_APP_BME280_I2C_ADDR,
        .scl_speed_hz = CONFIG_APP_I2C_FREQ_HZ,
        .mode = BME280_MODE_FORCED,
        .temp_over_sample = BME280_OVER_SAMPLE_1X,
        .hum_over_sample = BME280_OVER_SAMPLE_1X,
        .pres_over_sample = BME280_OVER_SAMPLE_1X,
        .filter = BME280_FILTER_OFF,
    };
    ESP_RETURN_ON_ERROR(bme280_init(&bme_cfg, &bme280_handle), TAG, "bme280 init failed");

    vcnl4010_config_t vcnl_cfg = {
        .bus_handle = bus_handle,
        .i2c_address = CONFIG_APP_VCNL4010_I2C_ADDR,
        .scl_speed_hz = CONFIG_APP_I2C_FREQ_HZ,
        .self_timed = true,
        .prox_enabled = true,
        .led_current = VCNL4010_LED_CURRENT_100mA,
        .prox_rate = VCNL4010_PROX_RATE_15_625,
        .interrupt = {
            .count = VCNL4010_INT_COUNT_1,
            .enable_threshold = true,
            .low_threshold = 0,
            .high_threshold = CONFIG_APP_VCNL4010_PROX_THRESHOLD,
        },
    };
    ESP_RETURN_ON_ERROR(vcnl4010_init(&vcnl_cfg, &vcnl4010_handle), TAG, "vcnl4010 init failed");

    uint8_t dummy_status = 0;
    ESP_RETURN_ON_ERROR(vcnl4010_clear_interrupt(vcnl4010_handle, &dummy_status), TAG, "clear int failed");

    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(CONFIG_APP_VCNL4010_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio config failed");

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_APP_VCNL4010_INT_GPIO, vcnl4010_isr_handler, NULL), TAG, "isr add failed");

    BaseType_t ret = xTaskCreate(presence_task, "presence_task", 3072, NULL, 5, &presence_task_handle);

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}