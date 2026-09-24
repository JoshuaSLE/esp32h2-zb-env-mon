#include "sensor_manager.h"
#include "bme280.h"
#include "display.h"
#include "vcnl4010.h"
#include "vcnl4010_def.h"
#include "zigbee.h"

#include "driver/gpio.h"
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task.h"

static const char *TAG = "sensor_manager";

static TaskHandle_t presence_task_handle = NULL;

static portMUX_TYPE reading_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool reading_dirty = false;

static bme280_handle_t bme280_handle = NULL;
static vcnl4010_handle_t vcnl4010_handle = NULL;
static esp_lcd_panel_handle_t display_handle = NULL;

static uint32_t display_timeout_ms = CONFIG_APP_DISPLAY_TIMEOUT_MS;
static bme280_data_t cached_reading = {0};

static void IRAM_ATTR vcnl4010_isr_handler(void *arg)
{
    gpio_intr_disable(CONFIG_APP_VCNL4010_INT_GPIO);

    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(presence_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

// ---------------------------------------------------------------------------
// Periodic BME280 Sampling Task
// ---------------------------------------------------------------------------
static void bme280_task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sample_interval = pdMS_TO_TICKS(CONFIG_APP_BME280_READ_INTERVAL_MS);

    while (1)
    {
        vTaskDelayUntil(&last_wake_time, sample_interval);

        esp_err_t err = bme280_trigger_measurement(bme280_handle);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "bme280 trigger failed: %s", esp_err_to_name(err));
            continue;
        }

        bme280_data_t reading;
        err = bme280_read_data(bme280_handle, &reading);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "bme280 read failed: %s", esp_err_to_name(err));
            continue;
        }

        portENTER_CRITICAL(&reading_lock);
        cached_reading = reading;
        reading_dirty = true;
        portEXIT_CRITICAL(&reading_lock);

        zigbee_report_bme280(&reading);

        if (presence_task_handle)
        {
            xTaskNotifyGive(presence_task_handle);
        }
    }
}

// ---------------------------------------------------------------------------
// Event-Driven Motion & Display Task
// ---------------------------------------------------------------------------
static void presence_task(void *pvParameters)
{
    TickType_t display_off_target_tick = 0;
    bool display_is_on = true;

    // Initial boot read (unchanged)
    if (bme280_trigger_measurement(bme280_handle) == ESP_OK)
    {
        bme280_data_t reading;
        if (bme280_read_data(bme280_handle, &reading) == ESP_OK)
        {
            portENTER_CRITICAL(&reading_lock);
            cached_reading = reading;
            reading_dirty = false;
            portEXIT_CRITICAL(&reading_lock);

            display_show_readings(display_handle, reading.temp, reading.hum, reading.press);
        }
    }

    display_off_target_tick = xTaskGetTickCount() + pdMS_TO_TICKS(display_timeout_ms);

    while (1)
    {
        TickType_t wait_ticks = display_is_on ? pdMS_TO_TICKS(500) : portMAX_DELAY;
        uint32_t notified = ulTaskNotifyTake(pdTRUE, wait_ticks);
        TickType_t now_ticks = xTaskGetTickCount();

        uint8_t int_status = 0;
        bool motion_detected = false;

        if (notified > 0)
        {
            esp_err_t int_err = vcnl4010_clear_interrupt(vcnl4010_handle, &int_status);
            gpio_intr_enable(CONFIG_APP_VCNL4010_INT_GPIO);
            motion_detected = (int_err == ESP_OK) && (int_status & VCNL4010_INT_STATUS_HIGH);
        }

        if (motion_detected)
        {
            if (!display_is_on)
            {
                display_on(display_handle);
                display_is_on = true;

                portENTER_CRITICAL(&reading_lock);
                reading_dirty = true;
                portEXIT_CRITICAL(&reading_lock);
            }

            display_off_target_tick = now_ticks + pdMS_TO_TICKS(display_timeout_ms);
        }

        if (display_is_on)
        {
            bool need_redraw;
            bme280_data_t reading_copy;

            portENTER_CRITICAL(&reading_lock);
            need_redraw = reading_dirty;
            reading_copy = cached_reading;
            reading_dirty = false;
            portEXIT_CRITICAL(&reading_lock);

            if (need_redraw)
            {
                display_show_readings(display_handle, reading_copy.temp, reading_copy.hum, reading_copy.press);
            }
        }

        if (display_is_on && (now_ticks >= display_off_target_tick))
        {
            if (display_off(display_handle) == ESP_OK)
            {
                display_is_on = false;
            }
        }
    }
}

esp_err_t sensor_manager_init(i2c_master_bus_handle_t bus_handle,
                              esp_lcd_panel_handle_t panel_handle)
{
    if (!bus_handle || !panel_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    display_handle = panel_handle;

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
        .scl_speed_hz = CONFIG_APP_I2C_FREQ_HZ,
        .self_timed = true,
        .prox_enabled = true,
        .led_current = VCNL4010_LED_CURRENT_100mA,
        .prox_rate = VCNL4010_PROX_RATE_15_625,
        .interrupt = {
            .count = VCNL4010_INT_COUNT_4,
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
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio config failed");

    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(CONFIG_APP_VCNL4010_INT_GPIO, GPIO_INTR_LOW_LEVEL),
                        TAG, "gpio wakeup enable failed");
    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(),
                        TAG, "sleep gpio wakeup enable failed");

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_APP_VCNL4010_INT_GPIO, vcnl4010_isr_handler, NULL), TAG, "isr add failed");

    BaseType_t ret1 = xTaskCreate(presence_task, "presence_task", 3072, NULL, 5, &presence_task_handle);
    BaseType_t ret2 = xTaskCreate(bme280_task, "bme280_task", 3072, NULL, 4, NULL);

    return (ret1 == pdPASS && ret2 == pdPASS) ? ESP_OK : ESP_FAIL;
}