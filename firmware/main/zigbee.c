#include "zigbee.h"
#include "alarm_timer.h"

#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

static const char *TAG = "zigbee";

#define ESP_ALARM_TIMER_SCHEDULE_MS 5000

#define ZCL_STRING_ATTR(buf_name, str_val, max_len) ({ \
    static char buf_name[max_len + 1];                 \
    size_t _len = strlen(str_val);                     \
    if (_len > max_len)                                \
        _len = max_len;                                \
    buf_name[0] = (char)_len;                          \
    memcpy(&buf_name[1], str_val, _len);               \
    buf_name;                                          \
})

#define ESP_ZIGBEE_STORAGE_PARTITION_NAME "nvs"
#define ENV_MONITOR_EP_ID 1
#define ESP_MANUFACTURER_NAME ZCL_STRING_ATTR(mfg_name, CONFIG_APP_ZB_MANUFACTURER_NAME, 32)
#define ESP_MODEL_IDENTIFIER ZCL_STRING_ATTR(model_id, CONFIG_APP_ZB_MODEL_IDENTIFIER, 32)

static volatile bool is_connected = false;

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
    {
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
    }
    break;

    case EZB_ZDO_SIGNAL_LEAVE:
    {
        is_connected = false;
        const ezb_zdo_signal_leave_params_t *leave_params = ezb_app_signal_get_params(app_signal);
        ESP_LOGI(TAG, "Left network successfully with type(0x%02x)", leave_params->leave_type);
    }
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
                is_connected = true;
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status(0x%02x), retrying initialization...", ezb_app_signal_to_string(signal_type), status);

            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_INITIALIZATION, ESP_ALARM_TIMER_SCHEDULE_MS);
        }
    }
    break;

    case EZB_BDB_SIGNAL_STEERING:
    {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));

        if (status == EZB_BDB_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Network steering completed");
            is_connected = true;
        }
        else
        {
            ESP_LOGW(TAG, "Failed to steering network with status(0x%02x)", status);

            alarm_timer_schedule(esp_zigbee_alarm_bdb_commissioning, EZB_BDB_MODE_NETWORK_STEERING, ESP_ALARM_TIMER_SCHEDULE_MS);
        }
    }
    break;

    default:
        ESP_LOGI(TAG, "Zigbee APP Signal: %s (type: 0x%02x)", ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }

    return true;
}

static esp_err_t create_data_model(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    if (!dev_desc)
    {
        ESP_LOGE(TAG, "Failed to create device descriptor");
        return ESP_ERR_NO_MEM;
    }

    ezb_af_ep_config_t ep_config = {
        .ep_id = ENV_MONITOR_EP_ID,
        .app_profile_id = EZB_AF_HA_PROFILE_ID,
        .app_device_id = 0xFFF0, // custom device ID
        .app_device_version = 0,
    };

    ezb_af_ep_desc_t ep_desc = ezb_af_create_endpoint_desc(&ep_config);
    if (!ep_desc)
    {
        ESP_LOGE(TAG, "Failed to create endpoint descriptor");
        ezb_af_free_device_desc(dev_desc);
        return ESP_ERR_NO_MEM;
    }

    /* Basic cluster */
    ezb_zcl_basic_cluster_server_config_t basic_cfg = {
        .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_UNKNOWN,
    };
    ezb_zcl_cluster_desc_t basic_desc = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                        (void *)ESP_MANUFACTURER_NAME);
    ezb_zcl_basic_cluster_desc_add_attr(basic_desc, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                        (void *)ESP_MODEL_IDENTIFIER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, basic_desc);

    /* Identify cluster */
    ezb_zcl_identify_cluster_server_config_t identify_cfg = {
        .identify_time = EZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };
    ezb_zcl_cluster_desc_t identify_desc = ezb_zcl_identify_create_cluster_desc(&identify_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, identify_desc);

    /* Temperature measurement */
    ezb_zcl_temperature_measurement_cluster_server_config_t temp_cfg = {
        .measured_value = 0,
        .min_measured_value = -4000, // -40.00 °C
        .max_measured_value = 8500,  //  85.00 °C
    };
    ezb_zcl_cluster_desc_t temp_desc = ezb_zcl_temperature_measurement_create_cluster_desc(&temp_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, temp_desc);

    /* Relative humidity measurement */
    ezb_zcl_rel_humidity_measurement_cluster_server_config_t hum_cfg = {
        .measured_value = 0,
        .min_measured_value = 0,
        .max_measured_value = 10000, // 100.00 %
    };
    ezb_zcl_cluster_desc_t hum_desc = ezb_zcl_rel_humidity_measurement_create_cluster_desc(&hum_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, hum_desc);

    /* Pressure measurement */
    ezb_zcl_pressure_measurement_cluster_server_config_t press_cfg = {
        .measured_value = 0,
        .min_measured_value = 300,  // 300 hPa
        .max_measured_value = 1100, // 1100 hPa
    };
    ezb_zcl_cluster_desc_t press_desc = ezb_zcl_pressure_measurement_create_cluster_desc(&press_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_af_endpoint_add_cluster_desc(ep_desc, press_desc);

    ezb_af_device_add_endpoint_desc(dev_desc, ep_desc);

    esp_err_t err = ezb_af_device_desc_register(dev_desc);
    if (err != EZB_ERR_NONE)
    {
        ESP_LOGE(TAG, "Failed to register device: %d", err);
        ezb_af_free_device_desc(dev_desc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esp_zigbee_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_RETURN_ON_ERROR(ezb_bdb_set_primary_channel_set((uint32_t)CONFIG_APP_ZB_PRIMARY_CHANNEL_MASK), TAG, "Failed to set the primary chanel mask");
    ESP_RETURN_ON_ERROR(ezb_bdb_set_secondary_channel_set((uint32_t)CONFIG_APP_ZB_SECONDARY_CHANNEL_MASK), TAG, "Failed to set the secondary chanel mask");
    ESP_RETURN_ON_ERROR(ezb_app_signal_add_handler(esp_zigbee_app_signal_handler),
                        TAG, "Failed to add the zigbee signal handler");
    ezb_nwk_set_rx_on_when_idle(false);
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

    ESP_ERROR_CHECK(create_data_model());

    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();

    vTaskDelete(NULL);
}

esp_err_t zigbee_init(void)
{
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "Failed to init the nvs flash partition");

    ESP_LOGI(TAG, "Starting ESP Zigbee Stack task...");
    BaseType_t ret = xTaskCreate(zigbee_stack_main_task, "Zigbee_main", 4096, NULL, 5, NULL);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Zigbee task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zigbee stack ready");

    return ESP_OK;
}

void zigbee_report_bme280(const bme280_data_t *reading)
{
    if (reading == NULL || !is_connected || isnan(reading->temp) || isnan(reading->hum) || isnan(reading->press))
    {
        ESP_LOGW(TAG, "Skip BME280 report");
        return;
    }

    if (reading->temp < -40.0f || reading->temp > 85.0f ||
        reading->hum < 0.0f || reading->hum > 100.0f ||
        reading->press < 300.0f || reading->press > 1100.0f)
    {
        ESP_LOGW(TAG, "Skip BME280 report: out of range T=%.2f H=%.2f P=%.2f",
                 reading->temp, reading->hum, reading->press);
        return;
    }

    esp_zigbee_lock_acquire(portMAX_DELAY);

    int16_t temp_zcl = (int16_t)(reading->temp * 100.0f);
    uint16_t hum_zcl = (uint16_t)(reading->hum * 100.0f);
    int16_t press_zcl = (int16_t)(reading->press);

    ezb_zcl_status_t temp_attr = ezb_zcl_set_attr_value(
        ENV_MONITOR_EP_ID, EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &temp_zcl, false);

    ezb_zcl_status_t hum_attr = ezb_zcl_set_attr_value(
        ENV_MONITOR_EP_ID, EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &hum_zcl, false);

    ezb_zcl_status_t press_attr = ezb_zcl_set_attr_value(
        ENV_MONITOR_EP_ID, EZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, EZB_ZCL_CLUSTER_SERVER,
        EZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MEASURED_VALUE_ID, EZB_ZCL_STD_MANUF_CODE, &press_zcl, false);

    if (temp_attr != EZB_ZCL_STATUS_SUCCESS || hum_attr != EZB_ZCL_STATUS_SUCCESS || press_attr != EZB_ZCL_STATUS_SUCCESS || !ezb_bdb_dev_joined())
    {
        esp_zigbee_lock_release();
        return;
    }

    ezb_zcl_report_attr_cmd_t report = {
        .cmd_ctrl = {
            .dst_addr.addr_mode = EZB_ADDR_MODE_NONE,
            .src_ep = ENV_MONITOR_EP_ID,
            .cluster_id = EZB_ZCL_CLUSTER_ID_TEMPERATURE_MEASUREMENT,
            .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
        },
        .payload = {
            .attr_id = EZB_ZCL_ATTR_TEMPERATURE_MEASUREMENT_MEASURED_VALUE_ID,
        },
    };

    ezb_zcl_report_attr_cmd_req(&report);

    report.cmd_ctrl.cluster_id = EZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT;
    report.payload.attr_id = EZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MEASURED_VALUE_ID;
    ezb_zcl_report_attr_cmd_req(&report);

    report.cmd_ctrl.cluster_id = EZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    report.payload.attr_id = EZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MEASURED_VALUE_ID;
    ezb_zcl_report_attr_cmd_req(&report);

    esp_zigbee_lock_release();
}