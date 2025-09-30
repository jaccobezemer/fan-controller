#include "Arduino.h"
#include "lvgl.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "pin_config.h"

#define MAX_BRIGHTNESS  4

// ======================= GLOBAAL =======================
static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static lv_display_t *disp = NULL;
static bool is_initialized_lvgl = false;

static lv_obj_t *fan_img;
static lv_anim_t fan_anim;

// ======================= IMAGE =======================
LV_IMG_DECLARE(fan_64x64);  // <-- Voeg hier je gegenereerde C-array toe

// LilyGo  T-Display-S3  control backlight chip has 16 levels of adjustment range
// The adjustable range is 0~16, 0 is the minimum brightness, 16 is the maximum brightness
void setBrightness(uint8_t value)
{
    static uint8_t level = 0;
    static uint8_t steps = 16;
    if (value == 0) {
        digitalWrite(PIN_LCD_BL, 0);
        delay(3);
        level = 0;
        return;
    }
    if (level == 0) {
        digitalWrite(PIN_LCD_BL, 1);
        level = steps;
        delayMicroseconds(30);
    }
    int from = steps - level;
    int to = steps - value;
    int num = (steps + to - from) % steps;
    for (int i = 0; i < num; i++) {
        digitalWrite(PIN_LCD_BL, 0);
        digitalWrite(PIN_LCD_BL, 1);
    }
    level = value;
}

// ======================= LVGL CALLBACKS =======================
static void example_lvgl_flush_cb(lv_display_t *disp_drv, const lv_area_t *area, uint8_t *color_map)
{
    if (!panel_handle) return;

    esp_lcd_panel_draw_bitmap(panel_handle,
                              area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1,
                              color_map);

    lv_display_flush_ready(disp_drv);
}

void lcd_panel_init(void)
{
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    Serial.begin(115200);

    // ===== LCD BUS INIT =====
    pinMode(PIN_LCD_RD, OUTPUT);
    digitalWrite(PIN_LCD_RD, HIGH);

    esp_lcd_i80_bus_handle_t i80_bus = NULL;

    esp_lcd_i80_bus_config_t bus_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .wr_gpio_num = PIN_LCD_WR,
        .clk_src = LCD_CLK_SRC_PLL160M,
        .data_gpio_nums = {
            PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3,
            PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7
        },
        .bus_width = 8,
        .max_transfer_bytes = LVGL_LCD_BUF_SIZE * sizeof(lv_color_t),
        .psram_trans_align = 0,
        .sram_trans_align = 0,
    };

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    // ===== IO CONFIG =====
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .user_ctx = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .swap_color_bytes = 1,
        },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    // ===== PANEL CONFIG =====
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RES,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 35));
}

// ======================= LVGL INIT =======================
void lvgl_display_init(void)
{
    lv_init();

    // Maak display object
    disp = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

    // Buffers aanmaken (2x partial)
    static lv_color_t buf1[LVGL_LCD_BUF_SIZE];
    static lv_color_t buf2[LVGL_LCD_BUF_SIZE];

    lv_display_set_buffers(disp, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Flush callback koppelen
    lv_display_set_flush_cb(disp, example_lvgl_flush_cb);

    // Tick callback koppelen (millis())
    lv_tick_set_cb([]() -> uint32_t { return millis(); });

    is_initialized_lvgl = true;
}

// ======================= FAN ANIMATIE =======================

static void fan_rotate_cb(void *obj, int32_t v)
{
    lv_image_set_rotation((lv_obj_t *)obj, v * 10);  // LVGL v9: 0.1° eenheden
}

void set_fan_speed(uint32_t duration_ms) {
   // Stop huidige animatie (indien actief)
    lv_anim_delete(fan_img, fan_rotate_cb); // Verwijder animatie voor fan_img met deze callback
    
    // Herinitialiseer animatie volledig
    lv_anim_init(&fan_anim);
    lv_anim_set_var(&fan_anim, fan_img);
    lv_anim_set_values(&fan_anim, 0, 360);
    lv_anim_set_duration(&fan_anim, duration_ms);
    lv_anim_set_repeat_count(&fan_anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_exec_cb(&fan_anim, fan_rotate_cb); // Expliciet callback opnieuw koppelen
    
    // Start animatie
    lv_anim_start(&fan_anim);
    lv_timer_handler();
}

void create_fan(void)
{
    fan_img = lv_image_create(lv_screen_active());
    lv_image_set_src(fan_img, &fan_64x64);
    lv_obj_center(fan_img);

    // Zet pivot in het midden
    lv_image_set_pivot(fan_img, fan_64x64.header.w / 2, fan_64x64.header.h / 2);

    // Anti-aliasing voor soepel draaien
    lv_image_set_antialias(fan_img, true);
}

// ======================= SETUP =======================
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("Start LVGL 9 Fan-demo...");
    lcd_panel_init();
    lvgl_display_init();

    // Fade backlight brightness to max configured brightness
    pinMode(PIN_LCD_BL, OUTPUT);
    // Brightness range : 0 ~ 16 level
    for (int i = 0; i <= MAX_BRIGHTNESS; ++i) {
        setBrightness(i);
        delay(100);
    }

    create_fan();
    set_fan_speed(250);
    Serial.println("Fan created");
    Serial.println("Start animation");
}

// ======================= LOOP =======================
void loop()
{
    static int step = 0;
    static bool accelerating = true;
    static unsigned long last_step_time = 0;
    // 10 stappen van 1500ms naar 250ms: (1500 - 250) / 9 ≈ 138.89ms per stap
    static uint32_t durations[] = {1500, 1361, 1222, 1083, 944, 805, 666, 527, 388, 250};

    // Wacht 2 seconden voordat de eerste versnelling begint
    if (millis() < 2000) {
        lv_task_handler();
        delay(5);
        return;
    }

    // Update snelheid elke 1000ms
    if (millis() - last_step_time >= 1000) {
        if (accelerating) {
            // Versnellen: 1500ms naar 250ms
            set_fan_speed(durations[step]);
            Serial.printf("Versnellen, stap %d, duur: %lu ms\n", step, durations[step]);
            step++;
            if (step >= 10) {
                step = 9; // Start vertragen vanaf 250ms (durations[9])
                accelerating = false;
            }
        } else {
            // Vertragen: 250ms naar 1500ms
            set_fan_speed(durations[step]);
            Serial.printf("Vertragen, stap %d, duur: %lu ms\n", step, durations[step]);
            step--;
            if (step < 0) {
                step = 0; // Start versnellen vanaf 1500ms (durations[0])
                accelerating = true;
            }
        }
        last_step_time = millis();
    }

    lv_task_handler(); // Verwerk LVGL events
    delay(5); // Kleine delay voor LVGL
}