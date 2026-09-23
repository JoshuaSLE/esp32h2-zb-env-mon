#include "vcnl4010.h"
#include "vcnl4010_def.h"

#include "esp_check.h"

static const char *TAG = "vcnl4010";

struct vcnl4010_handle
{
    i2c_master_dev_handle_t i2c_device;
};

static esp_err_t vcnl4010_reg_read(vcnl4010_handle_t handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle->i2c_device, &reg_addr, 1, data, len, CONFIG_APP_I2C_TIMEOUT_MS);
}

static esp_err_t vcnl4010_reg_write(vcnl4010_handle_t handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(handle->i2c_device, write_buf, sizeof(write_buf), CONFIG_APP_I2C_TIMEOUT_MS);
}

esp_err_t vcnl4010_init(const vcnl4010_config_t *config, vcnl4010_handle_t *ret_handle)
{
    if (config == NULL || ret_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    vcnl4010_handle_t dev = (vcnl4010_handle_t)calloc(1, sizeof(struct vcnl4010_handle));
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

    uint8_t product_id = 0;
    ESP_GOTO_ON_ERROR(vcnl4010_reg_read(dev, VCNL4010_REG_PRODUCT_ID, &product_id, 1), fail, TAG, "Failed to read product id");

    if ((product_id & 0xF0) != (VCNL4010_PRODUCT_ID & 0xF0))
    {
        ret = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_PROX_RATE, config->prox_rate), fail, TAG, "Failed to set proximity rate");

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_LED_CURRENT, config->led_current), fail, TAG, "Failed to set LED current");

    uint8_t als_param = ((config->als_rate & 0x07) << 4) |
                        (config->als_auto_offset ? VCNL4010_ALS_AUTO_OFFSET_EN : 0) |
                        (config->als_average & 0x07);
    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_ALS_PARAM, als_param), fail, TAG, "Failed to set ambient light parameters");

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_LOW_THRES, (config->interrupt.low_threshold >> 8) & 0xFF), fail, TAG, "Failed to set low threshold high bite");

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_LOW_THRES + 1, config->interrupt.low_threshold & 0xFF), fail, TAG, "Failed to set low threshold low bite");

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_HIGH_THRES, (config->interrupt.high_threshold >> 8) & 0xFF), fail, TAG, "Failed to set high threshold high bite");

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_HIGH_THRES + 1, config->interrupt.high_threshold & 0xFF), fail, TAG, "Failed to set high threshold low bite");

    uint8_t int_ctrl = ((config->interrupt.count & 0x07) << 5);
    if (config->interrupt.enable_threshold)
        int_ctrl |= VCNL4010_INT_THRES_EN;
    if (config->interrupt.enable_als_ready)
        int_ctrl |= VCNL4010_INT_ALS_READY;
    if (config->interrupt.enable_prox_ready)
        int_ctrl |= VCNL4010_INT_PROX_READY;

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_INT_CTRL, int_ctrl), fail, TAG, "Failed to set interrupt control");

    uint8_t cmd = 0;
    if (config->self_timed)
        cmd |= VCNL4010_CMD_SELFTIMED_EN;
    if (config->prox_enabled)
        cmd |= VCNL4010_CMD_PROX_EN;
    if (config->als_enabled)
        cmd |= VCNL4010_CMD_ALS_EN;

    ESP_GOTO_ON_ERROR(vcnl4010_reg_write(dev, VCNL4010_REG_COMMAND, cmd), fail, TAG, "Failed to set the comand");

    *ret_handle = dev;
    return ESP_OK;

fail:
    (void)i2c_master_bus_rm_device(dev->i2c_device);
    free(dev);
    return ret;
}

esp_err_t vcnl4010_deinit(vcnl4010_handle_t *handle)
{
    if (handle == NULL || *handle == NULL)
    {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device((*handle)->i2c_device), "vcnl4010", "failed to remove vcnl4010");

    free(*handle);
    *handle = NULL;

    return ESP_OK;
}

esp_err_t vcnl4010_clear_interrupt(vcnl4010_handle_t handle, uint8_t *status_out)
{
    if (handle == NULL || status_out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(vcnl4010_reg_read(handle, VCNL4010_REG_INT_STATUS, &status, 1), TAG, "failed reading int status");
    ESP_RETURN_ON_ERROR(vcnl4010_reg_write(handle, VCNL4010_REG_INT_STATUS, status), TAG, "failed clearing int status");
    if (status_out)
        *status_out = (status & 0x0F);
    return ESP_OK;
}