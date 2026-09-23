#include "bme280.h"
#include "bme280_defs.h"

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#define bme280_wait_until_ready(handle, timeout) \
    bme280_wait_status_bit_cleared(handle, BME280_STATUS_UPDATE, timeout)

#define bme280_wait_until_measuring_done(handle, timeout) \
    bme280_wait_status_bit_cleared(handle, BME280_STATUS_MEAS, timeout)

static const char *TAG = "bme280";

typedef struct bme280_comp
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;

    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
} bme280_comp_t;

struct bme280_handle
{
    i2c_master_dev_handle_t i2c_device;
    bme280_comp_t comp;
    int32_t t_fine;
    bme280_mode_t mode;
    uint8_t ctrl_meas;
};

static esp_err_t bme280_reg_read(bme280_handle_t handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle->i2c_device, &reg_addr, 1, data, len, CONFIG_APP_I2C_TIMEOUT_MS);
}

static esp_err_t bme280_reg_write(bme280_handle_t handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(handle->i2c_device, write_buf, sizeof(write_buf), CONFIG_APP_I2C_TIMEOUT_MS);
}

static esp_err_t bme280_wait_status_bit_cleared(bme280_handle_t handle, uint8_t bit_mask, uint32_t timeout_ms)
{
    TickType_t start_ticks = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    uint8_t status = 0;

    do
    {
        ESP_RETURN_ON_ERROR(bme280_reg_read(handle, BME280_REG_STATUS, &status, 1), TAG, "failed reading status");

        if ((status & bit_mask) == 0)
        {
            return ESP_OK;
        }

        vTaskDelay(1);
    } while ((xTaskGetTickCount() - start_ticks) < timeout_ticks);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t bme280_wait_status_bit_set(bme280_handle_t handle, uint8_t bit_mask, uint32_t timeout_ms)
{
    TickType_t start_ticks = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    uint8_t status = 0;

    do
    {
        ESP_RETURN_ON_ERROR(bme280_reg_read(handle, BME280_REG_STATUS, &status, 1), TAG, "failed reading status");

        if ((status & bit_mask) != 0)
        {
            return ESP_OK;
        }

        esp_rom_delay_us(500);
    } while ((xTaskGetTickCount() - start_ticks) < timeout_ticks);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t bme280_read_calibration_data(bme280_handle_t handle)
{
    uint8_t calib1[26];
    uint8_t calib2[7];

    ESP_RETURN_ON_ERROR(bme280_reg_read(handle, BME280_REG_CALIB00, calib1, 26), TAG, "failed reading calib1");

    handle->comp.dig_T1 = (uint16_t)(calib1[1] << 8 | calib1[0]);
    handle->comp.dig_T2 = (int16_t)(calib1[3] << 8 | calib1[2]);
    handle->comp.dig_T3 = (int16_t)(calib1[5] << 8 | calib1[4]);

    handle->comp.dig_P1 = (uint16_t)(calib1[7] << 8 | calib1[6]);
    handle->comp.dig_P2 = (int16_t)(calib1[9] << 8 | calib1[8]);
    handle->comp.dig_P3 = (int16_t)(calib1[11] << 8 | calib1[10]);
    handle->comp.dig_P4 = (int16_t)(calib1[13] << 8 | calib1[12]);
    handle->comp.dig_P5 = (int16_t)(calib1[15] << 8 | calib1[14]);
    handle->comp.dig_P6 = (int16_t)(calib1[17] << 8 | calib1[16]);
    handle->comp.dig_P7 = (int16_t)(calib1[19] << 8 | calib1[18]);
    handle->comp.dig_P8 = (int16_t)(calib1[21] << 8 | calib1[20]);
    handle->comp.dig_P9 = (int16_t)(calib1[23] << 8 | calib1[22]);

    handle->comp.dig_H1 = calib1[25];

    ESP_RETURN_ON_ERROR(bme280_reg_read(handle, BME280_REG_CALIB26, calib2, 7), TAG, "failed reading calib2");

    handle->comp.dig_H2 = (int16_t)(calib2[1] << 8 | calib2[0]);
    handle->comp.dig_H3 = calib2[2];
    handle->comp.dig_H4 = (int16_t)(((int8_t)calib2[3] << 4) | (calib2[4] & 0x0F));
    handle->comp.dig_H5 = (int16_t)(((int8_t)calib2[5] << 4) | (calib2[4] >> 4));
    handle->comp.dig_H6 = (int8_t)calib2[6];

    return ESP_OK;
}

static int32_t bme280_comp_temp(bme280_handle_t handle, int32_t adc_T)
{
    int32_t var1, var2, temp;
    int32_t temp_min = -4000;
    int32_t temp_max = 8500;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)handle->comp.dig_T1 * 2));
    var1 = (var1 * ((int32_t)handle->comp.dig_T2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)handle->comp.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)handle->comp.dig_T3)) / 16384;

    handle->t_fine = var1 + var2;
    temp = (handle->t_fine * 5 + 128) / 256;

    if (temp < temp_min)
        temp = temp_min;
    else if (temp > temp_max)
        temp = temp_max;

    return temp;
}

static uint32_t bme280_comp_hum(bme280_handle_t handle, int32_t adc_H)
{
    int32_t var1, var2, var3, var4, var5;
    uint32_t hum;
    uint32_t hum_max = 102400;

    var1 = handle->t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)handle->comp.dig_H4) * 1048576);
    var4 = ((int32_t)handle->comp.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)handle->comp.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)handle->comp.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)handle->comp.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)handle->comp.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    hum = (uint32_t)(var5 / 4096);

    if (hum > hum_max)
        hum = hum_max;

    return hum;
}

static uint32_t bme280_comp_press(bme280_handle_t handle, int32_t adc_P)
{
    int32_t var1, var2;
    uint32_t p;

    var1 = (((int32_t)handle->t_fine) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)handle->comp.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)handle->comp.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)handle->comp.dig_P4) << 16);
    var1 = (((handle->comp.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)handle->comp.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)handle->comp.dig_P1)) >> 15);
    if (var1 == 0)
        return 0;

    p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;

    if (p < 0x80000000)
        p = (p << 1) / ((uint32_t)var1);
    else
        p = (p / (uint32_t)var1) * 2;

    var1 = (((int32_t)handle->comp.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)handle->comp.dig_P8)) >> 13;

    p = (uint32_t)((int32_t)p + ((var1 + var2 + handle->comp.dig_P7) >> 4));
    return p;
}

esp_err_t bme280_init(const bme280_config_t *config, bme280_handle_t *ret_handle)
{
    if (config == NULL || ret_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    bme280_handle_t dev = (bme280_handle_t)calloc(1, sizeof(struct bme280_handle));
    if (dev == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_address,
        .scl_speed_hz = config->scl_speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_cfg, &dev->i2c_device);
    if (ret != ESP_OK)
    {
        free(dev);
        return ret;
    }

    ESP_GOTO_ON_ERROR(bme280_reg_write(dev, BME280_REG_RESET, BME280_SOFT_RESET_VAL), fail, TAG, "Failed to soft reset");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t chip_id = 0;
    ESP_GOTO_ON_ERROR(bme280_reg_read(dev, BME280_REG_ID, &chip_id, 1), fail, TAG, "Failed to read the chip id");
    if (chip_id != BME280_ID_VAL)
    {
        ret = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    ESP_GOTO_ON_ERROR(bme280_wait_until_ready(dev, CONFIG_APP_I2C_TIMEOUT_MS), fail, TAG, "Failed to wait for is to be ready");

    ESP_GOTO_ON_ERROR(bme280_read_calibration_data(dev), fail, TAG, "Failed to read the calibration values");

    ESP_GOTO_ON_ERROR(bme280_reg_write(dev, BME280_REG_CTRL_HUM, config->hum_over_sample & 0x07), fail, TAG, "Failed to configure the humidity over sampling");

    uint8_t config_reg = ((config->standby & 0x07) << 5) | ((config->filter & 0x07) << 2);
    ESP_GOTO_ON_ERROR(bme280_reg_write(dev, BME280_REG_CONFIG, config_reg), fail, TAG, "Failed to configure the configuration");

    dev->mode = config->mode;
    dev->ctrl_meas = ((config->temp_over_sample & 0x07) << 5) |
                     ((config->pres_over_sample & 0x07) << 2) |
                     (dev->mode & 0x03);
    ESP_GOTO_ON_ERROR(bme280_reg_write(dev, BME280_REG_CTRL_MEAS, dev->ctrl_meas), fail, TAG, "Failed to configure the measure");

    *ret_handle = dev;
    return ESP_OK;

fail:
    (void)i2c_master_bus_rm_device(dev->i2c_device);
    free(dev);
    return ret;
}

esp_err_t bme280_deinit(bme280_handle_t *handle)
{
    if (handle == NULL || *handle == NULL)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device((*handle)->i2c_device), TAG, "failed to remove bme280");

    free(*handle);
    *handle = NULL;

    return ESP_OK;
}

esp_err_t bme280_trigger_measurement(bme280_handle_t handle)
{
    if (handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->mode != BME280_MODE_FORCED)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(bme280_reg_write(handle, BME280_REG_CTRL_MEAS, handle->ctrl_meas),
                        TAG, "failed to trigger forced measurement");

    ESP_RETURN_ON_ERROR(bme280_wait_status_bit_set(handle, BME280_STATUS_MEAS, 20),
                        TAG, "conversion never started");

    return bme280_wait_status_bit_cleared(handle, BME280_STATUS_MEAS, CONFIG_APP_I2C_TIMEOUT_MS);
}

esp_err_t bme280_read_data(bme280_handle_t handle, bme280_data_t *data)
{
    if (handle == NULL || data == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[8];
    ESP_RETURN_ON_ERROR(bme280_reg_read(handle, BME280_REG_PRESS, raw, 8), TAG, "failed reading raw sensor data");

    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)raw[6] << 8) | ((uint32_t)raw[7]));

    int32_t temp_raw = bme280_comp_temp(handle, adc_T);
    uint32_t press_raw = bme280_comp_press(handle, adc_P);
    uint32_t hum_raw = bme280_comp_hum(handle, adc_H);

    data->temp = temp_raw / 100.0f;   // °C
    data->press = press_raw / 100.0f; // hPa
    data->hum = hum_raw / 1024.0f;    // %RH

    return ESP_OK;
}