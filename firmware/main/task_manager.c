#include "task_manager.h"
#include "bme280.h"
#include "display.h"
#include "vcnl4010.h"
#include "vcnl4010_def.h"
#include "zigbee.h"

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "task_manager";

#define DISP_EVT_NEW_READING BIT0
#define DISP_EVT_MOTION BIT1

#define LONG_PRESS_MS 3000
#define DEBOUNCE_MS 50
#define POLL_MS 50

/* ---------------- Shared state ---------------- */

typedef struct cached_reading
{
    bool valid;
    bme280_data_t data;
} cached_reading_t;

static cached_reading_t cached;
static portMUX_TYPE reading_lock = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t display_timeout_ms = 10000;

static bme280_handle_t bme280_handle = NULL;
static vcnl4010_handle_t vcnl4010_handle = NULL;
static esp_lcd_panel_handle_t display_handle = NULL;

static TaskHandle_t vcnl_task_handle = NULL;
static TaskHandle_t display_task_handle = NULL;
static TaskHandle_t rst_task_handle = NULL;
static TaskHandle_t timeout_task_handle = NULL;

/* ---------------- Helpers ---------------- */

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

static void display_paint_cached(void)
{
    cached_reading_t snap = cached_snapshot();
    if (!snap.valid)
    {
        return;
    }
    display_update(display_handle, snap.data.temp, snap.data.hum, snap.data.press,
                   display_timeout_ms / 1000);
}

/* ---------------- Generic button ISR ---------------- */
/*
 * All three "buttons" (VCNL INT, reset button, timeout button) do exactly
 * the same thing in their ISR: disable their own interrupt, poke a task.
 * One ISR + a context struct replaces three copies.
 */
typedef struct
{
    int pin;
    TaskHandle_t task;
} button_isr_ctx_t;

static button_isr_ctx_t vcnl_isr_ctx;
static button_isr_ctx_t rst_isr_ctx;
static button_isr_ctx_t timeout_isr_ctx;

static void IRAM_ATTR button_isr(void *arg)
{
    const button_isr_ctx_t *ctx = (const button_isr_ctx_t *)arg;

    gpio_intr_disable(ctx->pin);

    BaseType_t hp_woken = pdFALSE;
    vTaskNotifyGiveFromISR(ctx->task, &hp_woken);
    portYIELD_FROM_ISR(hp_woken);
}

/* ---------------- Reset / pairing button ---------------- */
static void rst_button_task(void *pv)
{
    (void)pv;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

        if (gpio_get_level(CONFIG_APP_INPUT_RST_GPIO) != 1)
        {
            gpio_intr_enable(CONFIG_APP_INPUT_RST_GPIO);
            continue;
        }

        TickType_t start_tick = xTaskGetTickCount();
        bool long_press_executed = false;

        /* Hold-detection loop: waits for release while polling for long-press threshold */
        while (gpio_get_level(CONFIG_APP_INPUT_RST_GPIO) == 1)
        {
            if (!long_press_executed && pdTICKS_TO_MS(xTaskGetTickCount() - start_tick) >= LONG_PRESS_MS)
            {
                long_press_executed = true;
                ESP_LOGW(TAG, "Reset button long press: leaving network & resetting");
                zigbee_factory_reset();
            }
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        }

        gpio_intr_enable(CONFIG_APP_INPUT_RST_GPIO);
    }
}

static void timeout_button_task(void *pv)
{
    (void)pv;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));

        if (gpio_get_level(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO) != 0)
        {
            gpio_intr_enable(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO);
            continue;
        }

        switch (display_timeout_ms)
        {
        case 10000:
            display_timeout_ms = 30000;
            break;
        case 30000:
            display_timeout_ms = 60000;
            break;
        default:
            display_timeout_ms = 10000;
            break;
        }

        ESP_LOGI(TAG, "Display timeout cycled to %us", (unsigned)(display_timeout_ms / 1000));

        /* Wake the display so the user sees the new timeout immediately. */
        xTaskNotify(display_task_handle, DISP_EVT_MOTION, eSetBits);

        /* Wait for release before re-arming the level interrupt. */
        while (gpio_get_level(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO) == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        }

        gpio_intr_enable(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO);
    }
}

static void vcnl_task(void *pv)
{
    (void)pv;

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

static void bme280_task(void *pv)
{
    (void)pv;

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
        xTaskNotify(display_task_handle, DISP_EVT_NEW_READING, eSetBits);
        zigbee_report_bme280(&reading);
    }
}

static void display_task(void *pv)
{
    (void)pv;

    bool display_is_on = false;
    TickType_t off_target = 0;

    /* Initial paint. If the boot read failed, cached.valid is false and
       the panel stays off until the first reading or a motion event. */
    if (cached_snapshot().valid)
    {
        display_paint_cached();
        display_is_on = true;
        off_target = xTaskGetTickCount() + pdMS_TO_TICKS(display_timeout_ms);
    }

    while (1)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t wait = display_is_on
                              ? ((off_target > now) ? (off_target - now) : 1)
                              : portMAX_DELAY;

        uint32_t events = 0;
        xTaskNotifyWait(0, UINT32_MAX, &events, wait);
        now = xTaskGetTickCount();

        if (events & DISP_EVT_MOTION)
        {
            if (!display_is_on && display_on(display_handle) == ESP_OK)
            {
                display_is_on = true;
            }
            if (display_is_on)
            {
                display_paint_cached();
            }
            off_target = now + pdMS_TO_TICKS(display_timeout_ms);
        }

        if ((events & DISP_EVT_NEW_READING) && display_is_on)
        {
            display_paint_cached();
        }

        if (display_is_on && now >= off_target)
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

    /* -------- BME280 -------- */
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

    /* One initial reading so display_task has something to show at boot. */
    if (bme280_trigger_measurement(bme280_handle) == ESP_OK)
    {
        bme280_data_t reading;
        if (bme280_read_data(bme280_handle, &reading) == ESP_OK)
        {
            cached_store(&reading);
        }
    }

    /* -------- VCNL4010 -------- */
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
    ESP_RETURN_ON_ERROR(vcnl4010_init(&vcnl_cfg, &vcnl4010_handle), TAG, "vcnl4010 init failed");

    uint8_t dummy_status = 0;
    ESP_RETURN_ON_ERROR(vcnl4010_clear_interrupt(vcnl4010_handle, &dummy_status), TAG, "clear int failed");

    /* -------- ISR service (once, before any handler is added) -------- */
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE)
    {
        return isr_err;
    }

    /* -------- GPIO config (one table per pin, but same shape) -------- */

    const gpio_config_t vcnl_io = {
        .pin_bit_mask = BIT64(CONFIG_APP_VCNL4010_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_LOW_LEVEL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&vcnl_io), TAG, "vcnl gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(CONFIG_APP_VCNL4010_INT_GPIO, GPIO_INTR_LOW_LEVEL),
                        TAG, "vcnl wakeup enable failed");

    const gpio_config_t rst_io = {
        .pin_bit_mask = BIT64(CONFIG_APP_INPUT_RST_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&rst_io), TAG, "rst gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(CONFIG_APP_INPUT_RST_GPIO, GPIO_INTR_HIGH_LEVEL),
                        TAG, "rst wakeup enable failed");

    const gpio_config_t timeout_io = {
        .pin_bit_mask = BIT64(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_HIGH_LEVEL,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&timeout_io), TAG, "timeout gpio config failed");
    ESP_RETURN_ON_ERROR(gpio_wakeup_enable(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO, GPIO_INTR_HIGH_LEVEL),
                        TAG, "timeout wakeup enable failed");

    ESP_RETURN_ON_ERROR(esp_sleep_enable_gpio_wakeup(), TAG, "sleep gpio wakeup enable failed");

    /* -------- Tasks -------- */
    BaseType_t r1 = xTaskCreate(display_task, "display_task", 3072, NULL, 5, &display_task_handle);
    BaseType_t r2 = xTaskCreate(vcnl_task, "vcnl_task", 2048, NULL, 6, &vcnl_task_handle);
    BaseType_t r3 = xTaskCreate(bme280_task, "bme280_task", 3072, NULL, 4, NULL);
    BaseType_t r4 = xTaskCreate(rst_button_task, "rst_btn", 2048, NULL, 10, &rst_task_handle);
    BaseType_t r5 = xTaskCreate(timeout_button_task, "tmout_btn", 2048, NULL, 10, &timeout_task_handle);

    if (r1 != pdPASS || r2 != pdPASS || r3 != pdPASS || r4 != pdPASS || r5 != pdPASS)
    {
        return ESP_FAIL;
    }

    /* -------- ISR handlers -------- */
    vcnl_isr_ctx = (button_isr_ctx_t){.pin = CONFIG_APP_VCNL4010_INT_GPIO, .task = vcnl_task_handle};
    rst_isr_ctx = (button_isr_ctx_t){.pin = CONFIG_APP_INPUT_RST_GPIO, .task = rst_task_handle};
    timeout_isr_ctx = (button_isr_ctx_t){.pin = CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO, .task = timeout_task_handle};

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_APP_VCNL4010_INT_GPIO,
                                             button_isr, &vcnl_isr_ctx),
                        TAG, "vcnl isr add failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_APP_INPUT_RST_GPIO,
                                             button_isr, &rst_isr_ctx),
                        TAG, "rst isr add failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(CONFIG_APP_INPUT_DISP_TIMEOUT_BTN_GPIO,
                                             button_isr, &timeout_isr_ctx),
                        TAG, "timeout isr add failed");

    return ESP_OK;
}