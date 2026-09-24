#include "task_manager.h"
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_manager";

#define DISPLAY_TIMEOUT_MS 10000

/* Event bits posted to the display task */
#define DISP_EVT_NEW_READING BIT0
#define DISP_EVT_MOTION BIT1

typedef struct cached_reading
{
    bool valid;
    bme280_data_t data;
} cached_reading_t;

static TaskHandle_t vcnl_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;

static bme280_handle_t bme280_handle = NULL;
static vcnl4010_handle_t vcnl4010_handle = NULL;
static esp_lcd_panel_handle_t display_handle = NULL;

static cached_reading_t cached;
static portMUX_TYPE reading_lock = portMUX_INITIALIZER_UNLOCKED;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */
static cached_reading_t cached_snapshot(void)
{
    cached_reading_t snap;
    portENTER_CRITICAL(&reading_lock);
    snap = cached;
    portEXIT_CRITICAL(&reading_lock);
    return snap;
}

static void cached_store(const bme280_data_t *reading)
{
    portENTER_CRITICAL(&reading_lock);
    cached.data = *reading;
    cached.valid = true;
    portEXIT_CRITICAL(&reading_lock);
}

/* Paint whatever is currently cached (no-op if invalid). */
static void display_paint_cached(void)
{
    cached_reading_t snap = cached_snapshot();
    if (!snap.valid)
    {
        return;
    }
    display_show_readings(display_handle, snap.data.temp, snap.data.hum, snap.data.press);
}

/* ------------------------------------------------------------------ */
/* VCNL4010 ISR — wakes only vcnl_task                                 */
/* ------------------------------------------------------------------ */
static void IRAM_ATTR vcnl4010_isr_handler(void *arg)
{
    gpio_intr_disable(CONFIG_APP_VCNL4010_INT_GPIO);

    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(vcnl_task_handle, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* ------------------------------------------------------------------ */
/* bme280_task — periodic sample, cache, report, notify display        */
/* ------------------------------------------------------------------ */
static void bme280_task(void *pv)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(CONFIG_APP_BME280_READ_INTERVAL_MS);

    while (1)
    {
        vTaskDelayUntil(&last_wake, interval);

        if (bme280_trigger_measurement(bme280_handle) != ESP_OK)
        {
            ESP_LOGW(TAG, "bme280 trigger failed");
            continue;
        }

        bme280_data_t reading;
        if (bme280_read_data(bme280_handle, &reading) != ESP_OK)
        {
            ESP_LOGW(TAG, "bme280 read failed");
            continue;
        }

        cached_store(&reading);

        /* Display first so the UI stays responsive even if the Zigbee
           lock is momentarily contended. */
        xTaskNotify(display_task_handle, DISP_EVT_NEW_READING, eSetBits);

        zigbee_report_bme280(&reading);
    }
}

/* ------------------------------------------------------------------ */
/* vcnl_task — translate ISR wakeup into a motion event                */
/* ------------------------------------------------------------------ */
static void vcnl_task(void *pv)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t int_status = 0;
        esp_err_t err = vcnl4010_clear_interrupt(vcnl4010_handle, &int_status);

        gpio_intr_enable(CONFIG_APP_VCNL4010_INT_GPIO);

        if (err == ESP_OK && (int_status & VCNL4010_INT_STATUS_HIGH))
        {
            xTaskNotify(display_task_handle, DISP_EVT_MOTION, eSetBits);
        }
    }
}

/* ------------------------------------------------------------------ */
/* display_task — owns the panel, reacts to events and its own timeout */
/* ------------------------------------------------------------------ */
static void display_task(void *pv)
{
    bool display_is_on = false;
    TickType_t off_target = 0;

    /* Initial paint from the reading done in sensor_manager_init().
       If the boot read failed, cached.valid is false and we simply
       leave the panel off until a reading or a motion event arrives. */
    cached_reading_t snap = cached_snapshot();
    if (snap.valid)
    {
        if (display_show_readings(display_handle,
                                  snap.data.temp, snap.data.hum, snap.data.press) == ESP_OK)
        {
            display_is_on = true;
            off_target = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TIMEOUT_MS);
        }
    }

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait;

        if (display_is_on)
        {
            wait = (off_target > now) ? (off_target - now) : 1;
        }
        else
        {
            wait = portMAX_DELAY;
        }

        uint32_t events = 0;
        xTaskNotifyWait(0, UINT32_MAX, &events, wait);
        now = xTaskGetTickCount();

        /* --- Motion: turn on (if needed) and refresh --- */
        if (events & DISP_EVT_MOTION)
        {
            if (!display_is_on)
            {
                if (display_on(display_handle) == ESP_OK)
                {
                    display_is_on = true;
                }
            }

            if (display_is_on)
            {
                display_paint_cached();
            }

            off_target = now + pdMS_TO_TICKS(DISPLAY_TIMEOUT_MS);
        }

        /* --- New reading: only matters if the panel is already on --- */
        if ((events & DISP_EVT_NEW_READING) && display_is_on)
        {
            display_paint_cached();
        }

        /* --- Timeout: turn off --- */
        if (display_is_on && now >= off_target)
        {
            if (display_off(display_handle) == ESP_OK)
            {
                display_is_on = false;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public init                                                         */
/* ------------------------------------------------------------------ */
esp_err_t sensor_manager_init(i2c_master_bus_handle_t bus_handle,
                              esp_lcd_panel_handle_t panel_handle)
{
    if (!bus_handle || !panel_handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    display_handle = panel_handle;

    /* ---------------- BME280 ---------------- */
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
    ESP_RETURN_ON_ERROR(bme280_init(&bme_cfg, &bme280_handle),
                        TAG, "bme280 init failed");

    /* One initial reading so display_task has something to show at boot. */
    if (bme280_trigger_measurement(bme280_handle) == ESP_OK)
    {
        bme280_data_t reading;
        if (bme280_read_data(bme280_handle, &reading) == ESP_OK)
        {
            cached_store(&reading);
        }
    }

    /* ---------------- VCNL4010 ---------------- */
    vcnl4010_config_t vcnl_cfg = {
        .bus_handle = bus_handle,
        .scl_speed_hz = CONFIG_APP_I2C_FREQ_HZ,
        .self_timed = true,
        .prox_enabled = true,
        .led_current = VCNL4010_LED_CURRENT_20mA,
        .prox_rate = VCNL4010_PROX_RATE_1_95,
        .interrupt = {
            .count = VCNL4010_INT_COUNT_1,
            .enable_threshold = true,
            .low_threshold = 0,
            .high_threshold = CONFIG_APP_VCNL4010_PROX_THRESHOLD,
        },
    };
    ESP_RETURN_ON_ERROR(vcnl4010_init(&vcnl_cfg, &vcnl4010_handle),
                        TAG, "vcnl4010 init failed");

    uint8_t dummy_status = 0;
    ESP_RETURN_ON_ERROR(vcnl4010_clear_interrupt(vcnl4010_handle, &dummy_status),
                        TAG, "clear int failed");

    /* ---------------- GPIO for VCNL INT ---------------- */
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(CONFIG_APP_VCNL4010_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "gpio config failed");

    ESP_RETURN_ON_ERROR(
        gpio_wakeup_enable(CONFIG_APP_VCNL4010_INT_GPIO, GPIO_INTR_LOW_LEVEL),
        TAG, "gpio wakeup enable failed");

    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(),
                        TAG, "sleep gpio wakeup enable failed");

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }

    /* ---------------- Tasks ---------------- */
    BaseType_t r1 = xTaskCreate(display_task, "display_task", 3072, NULL, 5, &display_task_handle);
    BaseType_t r2 = xTaskCreate(vcnl_task, "vcnl_task", 2048, NULL, 6, &vcnl_task_handle);
    BaseType_t r3 = xTaskCreate(bme280_task, "bme280_task", 3072, NULL, 4, NULL);

    if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS)
    {
        return ESP_FAIL;
    }

    /* Attach the ISR last — vcnl_task_handle must be valid before any interrupt can fire. */
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(CONFIG_APP_VCNL4010_INT_GPIO, vcnl4010_isr_handler, NULL),
        TAG, "isr add failed");

    return ESP_OK;
}