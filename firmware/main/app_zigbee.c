#include "app_zigbee.h"
#include "alarm_timer.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_task.h"
#include "esp_zigbee.h"
#include "nvs_flash.h"

static const char *TAG = "app_zigbee";

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "nvs"

static void esp_zigbee_alarm_bdb_commissioning(alarm_timer_arg_t arg)
{
    esp_zigbee_lock_acquire(portMAX_DELAY);
    (void)ezb_bdb_start_top_level_commissioning(arg);
    esp_zigbee_lock_release();
}

static bool esp_zigbee_app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type)
    {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT:
    {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", ezb_bdb_is_factory_new() ? "" : "non-");
            if (ezb_bdb_is_factory_new())
            {
                ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Rebooted with existing network pairing");
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), retrying initialization...", ezb_app_signal_to_string(signal_type), status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, 1000);
        }
    }
    break;

    case EZB_BDB_SIGNAL_STEERING:
    {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS)
        {
            ezb_extpanid_t extended_pan_id;
            ezb_nwk_get_extended_panid(&extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully: PAN ID(0x%04hx, EXT: 0x%llx), Channel(%d), Short Address(0x%04hx)",
                     ezb_nwk_get_panid(), extended_pan_id.u64, ezb_nwk_get_current_channel(), ezb_nwk_get_short_address());
        }
        else
        {
            ESP_LOGW(TAG, "Network steering failed with status(0x%02x), retrying...", status);
            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
    }
    break;

    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s (type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_RETURN_ON_ERROR(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler), TAG, "Failed to add the zigbee signal handler");

    return ESP_OK;
}

static void zigbee_stack_main_task(void *pvParameters)
{
    esp_zigbee_config_t config = {
        .device_config = {
            .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,
            .install_code_policy = false,
            .zed_config = {
                .ed_timeout = EZB_NWK_ED_TIMEOUT_64MIN,
                .keep_alive = 3000,
            },
        },
        .platform_config = {
            .storage_partition_name = ESP_ZIGBEE_STORAGE_PARTITION_NAME,
            .radio_config = {
                .radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
            },
        },
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    ESP_ERROR_CHECK(esp_zigbee_setup_commissioning());

    /* Create and register the ZCL data model. */

    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();

    vTaskDelete(NULL);
}

esp_err_t zigbee_init(void)
{
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "Failed to init the nvs flash partition");

    ESP_LOGI(TAG, "Start ESP Zigbee Stack");
    BaseType_t ret = xTaskCreate(zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);

    return ret == pdPASS ? ESP_OK : ESP_FAIL;
}
