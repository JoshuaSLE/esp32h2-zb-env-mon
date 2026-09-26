#include "display.h"

#include <string.h>
#include <stdio.h>

#include "esp_check.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"

#define FB_WIDTH 128
#define FB_HEIGHT 64
#define FB_SIZE (FB_WIDTH * FB_HEIGHT / 8)
#define FONT_COUNT (sizeof(font_5x7) / sizeof(font_5x7[0]))

static const char *TAG = "display";
static uint8_t framebuffer[FB_SIZE];

typedef struct
{
    uint8_t ch;
    uint8_t cols[5];
} font_glyph_t;

static const font_glyph_t font_5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'%', {0x23, 0x13, 0x08, 0x64, 0x62}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7C, 0x12, 0x11, 0x12, 0x7C}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
    {'a', {0x20, 0x54, 0x54, 0x54, 0x78}},
    {'b', {0x7F, 0x48, 0x44, 0x44, 0x38}},
    {'c', {0x38, 0x44, 0x44, 0x44, 0x20}},
    {'d', {0x38, 0x44, 0x44, 0x48, 0x7F}},
    {'e', {0x38, 0x54, 0x54, 0x54, 0x18}},
    {'f', {0x08, 0x7E, 0x09, 0x01, 0x02}},
    {'g', {0x0C, 0x52, 0x52, 0x52, 0x3E}},
    {'h', {0x7F, 0x08, 0x04, 0x04, 0x78}},
    {'i', {0x00, 0x44, 0x7D, 0x40, 0x00}},
    {'j', {0x20, 0x40, 0x44, 0x3D, 0x00}},
    {'k', {0x7F, 0x10, 0x28, 0x44, 0x00}},
    {'l', {0x00, 0x41, 0x7F, 0x40, 0x00}},
    {'m', {0x7C, 0x04, 0x18, 0x04, 0x78}},
    {'n', {0x7C, 0x08, 0x04, 0x04, 0x78}},
    {'o', {0x38, 0x44, 0x44, 0x44, 0x38}},
    {'p', {0x7C, 0x14, 0x14, 0x14, 0x08}},
    {'q', {0x08, 0x14, 0x14, 0x18, 0x7C}},
    {'r', {0x7C, 0x08, 0x04, 0x04, 0x08}},
    {'s', {0x48, 0x54, 0x54, 0x54, 0x20}},
    {'t', {0x04, 0x3E, 0x44, 0x24, 0x08}},
    {'u', {0x3C, 0x40, 0x40, 0x20, 0x7C}},
    {'v', {0x1C, 0x20, 0x40, 0x20, 0x1C}},
    {'w', {0x3C, 0x40, 0x30, 0x40, 0x3C}},
    {'x', {0x44, 0x28, 0x10, 0x28, 0x44}},
    {'y', {0x0C, 0x50, 0x50, 0x50, 0x3C}},
    {'z', {0x44, 0x64, 0x54, 0x4C, 0x44}},
    {0xB0, {0x00, 0x06, 0x09, 0x06, 0x00}}, // '°'
};

static const font_glyph_t *font_find(uint8_t c)
{
    int lo = 0, hi = FONT_COUNT - 1;
    while (lo <= hi)
    {
        int mid = (lo + hi) / 2;
        if (font_5x7[mid].ch == c)
        {
            return &font_5x7[mid];
        }
        if (font_5x7[mid].ch < c)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
    return NULL;
}

static inline void fb_clear(void)
{
    memset(framebuffer, 0x00, sizeof(framebuffer));
}

static void fb_draw_char(int x, int y, uint8_t c)
{
    const font_glyph_t *g = font_find(c);
    if (!g || x < 0 || (x + 5) > FB_WIDTH || y < 0 || y >= FB_HEIGHT)
        return;

    int page = y / 8;
    int shift = y % 8;

    for (int col = 0; col < 5; col++)
    {
        uint8_t glyph_col = g->cols[col];
        size_t idx = (x + col) + page * FB_WIDTH;

        framebuffer[idx] |= (glyph_col << shift);
        if (shift > 1 && (page + 1) < (FB_HEIGHT / 8))
        {
            framebuffer[idx + FB_WIDTH] |= (glyph_col >> (8 - shift));
        }
    }
}

static void fb_draw_string(int x, int y, const char *s)
{
    const uint8_t *p = (const uint8_t *)s;
    while (*p)
    {
        fb_draw_char(x, y, *p++);
        x += 6;
    }
}

esp_err_t display_init(i2c_master_bus_handle_t bus_handle, esp_lcd_panel_handle_t *panel_handle)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = CONFIG_APP_SSD1306_I2C_ADDR,
        .scl_speed_hz = CONFIG_APP_I2C_FREQ_HZ,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus_handle, &io_config, &io_handle),
                        TAG, "failed to create lcd panel io");

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = FB_HEIGHT,
    };

    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
        .vendor_config = &ssd1306_config,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, panel_handle),
                        TAG, "failed to create ssd1306 panel");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*panel_handle), TAG, "failed to reset the panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*panel_handle), TAG, "failed to init the panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(*panel_handle, true, true), TAG, "panel mirror failed");

    // Clear the screen using the static framebuffer before turning it on
    fb_clear();
    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_draw_bitmap(*panel_handle, 0, 0, FB_WIDTH, FB_HEIGHT, framebuffer),
        TAG, "failed to push clear buffer to panel");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(*panel_handle, true), TAG, "failed to turn on the panel");

    return ESP_OK;
}

esp_err_t display_deinit(esp_lcd_panel_handle_t *panel_handle)
{
    if (panel_handle == NULL || *panel_handle == NULL)
    {
        return ESP_OK;
    }

    esp_err_t err = esp_lcd_panel_del(*panel_handle);
    if (err == ESP_OK)
    {
        *panel_handle = NULL;
    }

    return err;
}

esp_err_t display_update(esp_lcd_panel_handle_t panel_handle,
                         float temp_c, float humidity_pct, float pressure_hpa, uint8_t display_timeout_s)
{
    if (panel_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    char line1[16], line2[16], line3[16], line4[16], line5[16];

    (void)snprintf(line1, sizeof(line1), "Readings");
    (void)snprintf(line2, sizeof(line2), "%.1f\xB0"
                                         "C",
                   temp_c);
    (void)snprintf(line3, sizeof(line3), "%.0f%%", humidity_pct);
    (void)snprintf(line4, sizeof(line4), "%.0fhPa", pressure_hpa);
    (void)snprintf(line5, sizeof(line5), "Timeout: %us", display_timeout_s);

    fb_clear();
    fb_draw_string(0, 0, line1);
    fb_draw_string(0, 12, line2);
    fb_draw_string(0, 24, line3);
    fb_draw_string(0, 36, line4);
    fb_draw_string(0, 56, line5);

    ESP_RETURN_ON_ERROR(
        esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, FB_WIDTH, FB_HEIGHT, framebuffer),
        TAG, "failed to display readings");

    return ESP_OK;
}

esp_err_t display_off_on(esp_lcd_panel_handle_t panel_handle, bool off_on)
{
    if (panel_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel_handle, off_on), TAG, "failed to turn off the panel");

    return ESP_OK;
}