#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief VCNL410 handle.
     *
     */
    typedef struct vcnl4010_handle *vcnl4010_handle_t;

    /**
     * @brief VCNL410 proximity rate.
     *
     */
    typedef enum vcnl4010_prox_rate
    {
        VCNL4010_PROX_RATE_1_95,
        VCNL4010_PROX_RATE_3_90625,
        VCNL4010_PROX_RATE_7_8125,
        VCNL4010_PROX_RATE_15_625,
        VCNL4010_PROX_RATE_31_25,
        VCNL4010_PROX_RATE_62_5,
        VCNL4010_PROX_RATE_125,
        VCNL4010_PROX_RATE_250,
    } vcnl4010_prox_rate_t;

    /**
     * @brief VCNL410 led current for proximity.
     *
     */
    typedef enum vcnl4010_led_current
    {
        VCNL4010_LED_CURRENT_OFF,
        VCNL4010_LED_CURRENT_10mA,
        VCNL4010_LED_CURRENT_20mA,
        VCNL4010_LED_CURRENT_30mA,
        VCNL4010_LED_CURRENT_40mA,
        VCNL4010_LED_CURRENT_50mA,
        VCNL4010_LED_CURRENT_60mA,
        VCNL4010_LED_CURRENT_70mA,
        VCNL4010_LED_CURRENT_80mA,
        VCNL4010_LED_CURRENT_90mA,
        VCNL4010_LED_CURRENT_100mA,
        VCNL4010_LED_CURRENT_110mA,
        VCNL4010_LED_CURRENT_120mA,
        VCNL4010_LED_CURRENT_130mA,
        VCNL4010_LED_CURRENT_140mA,
        VCNL4010_LED_CURRENT_150mA,
        VCNL4010_LED_CURRENT_160mA,
        VCNL4010_LED_CURRENT_170mA,
        VCNL4010_LED_CURRENT_180mA,
        VCNL4010_LED_CURRENT_190mA,
        VCNL4010_LED_CURRENT_200mA,
    } vcnl4010_led_current_t;

    /**
     * @brief VCNL410 ambient light rate.
     *
     */
    typedef enum vcnl4010_als_rate
    {
        VCNL4010_ALS_RATE_1,
        VCNL4010_ALS_RATE_2,
        VCNL4010_ALS_RATE_3,
        VCNL4010_ALS_RATE_4,
        VCNL4010_ALS_RATE_5,
        VCNL4010_ALS_RATE_6,
        VCNL4010_ALS_RATE_8,
        VCNL4010_ALS_RATE_10,
    } vcnl4010_als_rate_t;

    /**
     * @brief VCNL410 ambient light avaraging.
     *
     */
    typedef enum vcnl4010_als_avg
    {
        VCNL4010_ALS_AVG_1,
        VCNL4010_ALS_AVG_2,
        VCNL4010_ALS_AVG_4,
        VCNL4010_ALS_AVG_8,
        VCNL4010_ALS_AVG_16,
        VCNL4010_ALS_AVG_32,
        VCNL4010_ALS_AVG_64,
        VCNL4010_ALS_AVG_128,
    } vcnl4010_als_avg_t;

    /**
     * @brief VCNL410 interrupt count.
     *
     */
    typedef enum vcnl4010_int_count
    {
        VCNL4010_INT_COUNT_1,
        VCNL4010_INT_COUNT_2,
        VCNL4010_INT_COUNT_4,
        VCNL4010_INT_COUNT_8,
        VCNL4010_INT_COUNT_16,
        VCNL4010_INT_COUNT_32,
        VCNL4010_INT_COUNT_64,
        VCNL4010_INT_COUNT_128,
    } vcnl4010_int_count_t;

    /**
     * @brief VCNL410 configuration.
     *
     */
    typedef struct vcnl4010_config
    {
        i2c_master_bus_handle_t bus_handle;
        uint32_t scl_speed_hz;

        bool self_timed;
        bool prox_enabled;
        bool als_enabled;

        vcnl4010_led_current_t led_current;
        vcnl4010_prox_rate_t prox_rate;
        vcnl4010_als_rate_t als_rate;
        vcnl4010_als_avg_t als_average;
        bool als_auto_offset;

        struct
        {
            vcnl4010_int_count_t count;
            bool enable_threshold;
            bool enable_als_ready;
            bool enable_prox_ready;
            uint16_t low_threshold;
            uint16_t high_threshold;
        } interrupt;
    } vcnl4010_config_t;

    /**
     * @brief VCNL4010 init.
     *
     * @param config VCNL4010 configuration.
     * @param ret_handle VCNL4010 handle.
     * @return esp_err_t
     */
    esp_err_t vcnl4010_init(const vcnl4010_config_t *config, vcnl4010_handle_t *ret_handle);

    /**
     * @brief VCNL4010 deinit.
     *
     * @param handle VCNL4010 handle.
     * @return esp_err_t
     */
    esp_err_t vcnl4010_deinit(vcnl4010_handle_t *handle);

    /**
     * @brief VCNL410 clear interrupt(s).
     *
     * @param handle VCNL4010 handle.
     * @param status_out Interrupt status register value.
     * @return esp_err_t
     */
    esp_err_t vcnl4010_clear_interrupt(vcnl4010_handle_t handle, uint8_t *status_out);

#ifdef __cplusplus
}
#endif