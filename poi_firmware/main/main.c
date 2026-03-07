#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "esp_random.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_sleep.h"

// Helper for animations (similar to FastLED)
#ifndef qsub8
#define qsub8(x, y) ((x>y)?x-y:0)
#endif

#ifndef scale8
#define scale8(i, scale) (((uint16_t)i * (1+(uint16_t)scale)) >> 8)
#endif

//debug

#include "esp_heap_caps.h"

//FS
#include "esp_littlefs.h"
static size_t storage_total = 0;
static size_t storage_used = 0;
static uint32_t free_space_kb = 0;
uint8_t file_h = 0; // The real height from the file
uint16_t img_w = 0;
volatile uint16_t bytes_in_buf = 0; 
volatile uint16_t buf_pos = 0;
uint8_t read_buf[1024]; // The physical buffer
//queue
#include "freertos/queue.h"
#include "freertos/semphr.h"

// --- Hardware ---
#define LED_GPIO            GPIO_NUM_6
#define BUTTON_GPIO         GPIO_NUM_3
#define REGULATOR_GPIO      GPIO_NUM_7
#define MAX_LEDS            20
// --- better calibrate the voltage readings individually for each poi,1: check the voltage reading in the web app, 2: measure with a multimeter
// then compare the values and adjust the scaling factor as needed. --- 
#define BATTERY_SCALING_FACTOR 3.06f

#define adcchan ADC_CHANNEL_2
// This is the actual value used in your LED loop on startup (updated via the index)
uint8_t g_brightness = 20;


typedef struct {
    uint8_t pkt_data[512];
    uint16_t len;
    bool is_final;      // Explicitly marks the end
    uint8_t skip_bytes; // To skip the 5-byte header in packet #1
} flash_packet_t;

QueueHandle_t flash_queue = NULL;

// NimBLE Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

uint16_t notify_handle;
uint16_t conn_hdl;

uint8_t tx_buffer[512];
uint16_t tx_len = 0;

/* Define UUIDs as actual static variables first */
static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static const ble_uuid128_t notify_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x04, 0x00, 0x40, 0x6E);

//advertising forward declaration
void start_advertising(void);
void bleSendFWVersion(void);
static const char *TAG = "Open Pixel Poi";

// speeds
// Predefined speed presets (delay calculations will use these)
// 0 = slowest, 255 = fastest
uint8_t g_speed_presets[6] = {1, 10, 50, 100, 230, 255};

// Which preset are we currently using? (0 to 5)
uint8_t g_selected_speed_index = 2; // Default to the 3rd speed

//brightness
// 6 Brightness Presets (0 to 255)
uint8_t g_brightness_presets[6] = {10, 40, 50, 60, 80, 100};
uint8_t g_selected_brightness_index = 1; // Default to a middle-high value



//shuffle stuff
// --- SHUFFLE GLOBALS ---
bool g_shuffle_all_banks = false;
bool g_shuffle_slots_only = false;
uint32_t g_shuffle_duration_ms = 5000; // Default 5 seconds
uint32_t g_last_shuffle_tick = 0;

// --- Button Logic State Machine ---
typedef enum {
    BS_INITIAL,
    BS_CLICK_DOWN,
    BS_CLICK_HOLD,
    BS_CLICK_HOLD_LONG,
    BS_CLICK_UP,
    BS_CLICK2_DOWN,
    BS_CLICK2_HOLD,
    BS_CLICK2_UP,
    BS_CLICK3_DOWN,
    BS_CLICK3_HOLD,
    BS_CLICK3_UP,
    BS_CLICK4_DOWN,
    BS_CLICK4_HOLD,
    BS_CLICK4_UP,
    BS_CLICK5_DOWN,
    BS_CLICK5_HOLD,
    BS_CLICK5_UP
} button_state_t;

typedef enum {
    DS_PATTERN,
    DS_WAITING,
    DS_WAITING2,
    DS_WAITING3,
    DS_WAITING4,
    DS_WAITING5,
    DS_VOLTAGE,
    DS_VOLTAGE2,
    DS_BANK,
    DS_BRIGHTNESS,
    DS_SPEED,
    DS_SHUTDOWN,
    DS_PATTERN_ALL,
    DS_PATTERN_ALL_ALL
} display_state_t;

// --- Button Globals ---
volatile bool g_btn_is_down = false;
volatile TickType_t g_btn_transition_tick = 0;
volatile TickType_t g_last_press_tick = 0; 

static button_state_t g_button_state = BS_INITIAL;
static display_state_t g_display_state = DS_PATTERN;
static uint32_t g_down_time = 0;
static uint32_t g_shut_down_at = 0;
static uint32_t g_display_state_last_updated = 0;

#define millis() (pdTICKS_TO_MS(xTaskGetTickCount()))

static void IRAM_ATTR button_isr_handler(void* arg) {
    bool current_level = gpio_get_level(BUTTON_GPIO);
    g_btn_is_down = !current_level;
    g_btn_transition_tick = xTaskGetTickCountFromISR();

    if (g_btn_is_down) {
        g_last_press_tick = g_btn_transition_tick;
    }
}

// Add the prototype for the flash animation if you don't have it
void run_flash_animation(uint32_t color);


//voltage
#include "esp_adc/adc_oneshot.h"
led_strip_handle_t led_strip; //declare here 
float pin_volt = 0;
int adc_raw = 0;
uint16_t to_send;

#define BATTERY_DIVIDER_RATIO  2.0f
float g_battery_voltage = 0.0f;
uint64_t g_last_battery_check = 0;
bool g_emergency_mode = false;


adc_oneshot_unit_handle_t adc1_handle;

float read_battery_voltage() {
    
    uint32_t adc_sum = 0;
    int samples = 16;

   for (int i = 0; i < samples; i++) {
        int temp;
        if (adc_oneshot_read(adc1_handle, adcchan, &temp) == ESP_OK) {
            adc_sum += temp;
        }
    }
    
    adc_raw = adc_sum / samples;
    float pin_volt = (adc_raw * BATTERY_SCALING_FACTOR) / 4095.0f;
    return pin_volt * BATTERY_DIVIDER_RATIO;
}
void show_sos_signal() {
    const uint8_t dim_red = 10; // Keep it low to save battery

    // --- 3 DOTS (S: . . .) ---
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, dim_red, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));

        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, 0, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(400)); // Gap between letters

    // --- 3 DASHES (O: --- --- ---) ---
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, dim_red, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(600)); // Dash is 3x Dot

        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, 0, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelay(pdMS_TO_TICKS(400)); // Gap between letters

    // --- 3 DOTS (S: . . .) ---
    for(int i = 0; i < 3; i++) {
        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, dim_red, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));

        for (int j = 0; j < MAX_LEDS; j++) {
            led_strip_set_pixel(led_strip, j, 0, 0, 0);
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

//LEDs stuff
#define READ_BUFFER_SIZE 1024
uint32_t frame_counter = 0;
volatile bool led_task_running = true;
volatile bool led_task_paused = false; // To tell us it's safely stopped

#define BYTES_PER_FRAME (MAX_LEDS * 3)
#define FRAME_COUNT 42
#define RING_BUF_SIZE (BYTES_PER_FRAME * FRAME_COUNT)

typedef enum {
    MODE_IDLE,
    MODE_PATTERN,   // Normal file-based playback
    MODE_STREAMING  // Active BLE streaming
} led_mode_t;

// Global state variable
// volatile ensures the LED task sees changes made in the BLE callback immediately
static volatile led_mode_t current_mode = MODE_PATTERN;

// Animation modes
static volatile uint8_t g_current_animation = 0; // 0 for default rainbow, 1 for Fire, 2 for Ocean, 3 for Custom

void fire_animation(led_strip_handle_t strip, uint8_t brightness);
void ocean_waves_animation(led_strip_handle_t strip, uint8_t brightness);
void custom_animation(led_strip_handle_t strip, uint8_t brightness);


//streaming
#define MAX_HZ 			500
#define LATENCY_MS      300

#define MAX_FRAMES      ((MAX_HZ * LATENCY_MS) / 1000)
#define FRAME_SIZE (MAX_LEDS * 3)   // 60 bytes
#define FRAMES_PER_PACKET 8              // How many frames to pack
#define TOTAL_DATA_LEN (FRAME_SIZE * FRAMES_PER_PACKET) // 480 bytes
#define MAX_BLE_PAYLOAD (TOTAL_DATA_LEN + 2)


static SemaphoreHandle_t frames_sem = NULL;
static int current_frame_idx = 0;
static bool is_writing_to_buffer = false;

typedef struct {
    uint8_t frames[MAX_FRAMES][FRAME_SIZE]; // dynamic
    volatile uint16_t head;
    volatile uint16_t tail;
    uint32_t total_played; // for global tracking
    bool is_streaming;
} pov_stream_buf_t;

static pov_stream_buf_t *ring_buf = NULL;
static TaskHandle_t pov_task_handle = NULL;
static esp_timer_handle_t pov_timer;
uint32_t current_period_us = 5000; // Default 200Hz (1/200s = 5000us)

//banks & slots
volatile uint8_t g_current_bank = 1;
volatile uint8_t g_current_slot = 1;



//FS stuff
void init_littlefs() {
    ESP_LOGI("LITTLEFS", "Initializing LittleFS...");

    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",          
        .partition_label = "storage",    
        .format_if_mount_failed = true,
        .dont_mount = false
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("LITTLEFS", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("LITTLEFS", "Failed to find LittleFS partition");
        } else {
            ESP_LOGE("LITTLEFS", "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    // Update global stats
    ret = esp_littlefs_info(conf.partition_label, &storage_total, &storage_used);
    if (ret == ESP_OK) {
        ESP_LOGI("LITTLEFS", "Partition size: total: %zu, used: %zu", storage_total, storage_used);
    }
}

// TX response buffer and responses
static uint8_t ble_resp_buf[20]; 
static uint16_t ble_resp_len = 0;

static const uint8_t resp_success[] = {0xD0, 0x00, 0x01}; // Standard Success
static const uint8_t resp_error[]   = {0xD0, 0x01, 0x01}; // Standard Error
static const uint8_t resp_firmware[]= {0xD0, 0x00, 0x06, 0x09, 0x02, 0xD1};

void set_ble_reply(const uint8_t *data, uint16_t len) {
    if (len <= sizeof(ble_resp_buf)) {
        memcpy(ble_resp_buf, data, len);
        ble_resp_len = len;
    }
}

// Save to file logic

bool g_multipart_active = false;
int current_active_slot = 0;       // Controlled by CC_SET_PATTERN_SLOT
uint8_t g_strip_height = 0;   // Extracted from the pattern data
uint16_t g_pattern_width = 0; // Extracted from the pattern data

// Janitor task
void update_storage_stats() {
    size_t total = 0, used = 0;
    // LittleFS uses the partition label defined in partitions.csv (e.g., "storage")
    esp_err_t ret = esp_littlefs_info("storage", &total, &used);
    
    if (ret == ESP_OK) {
        storage_total = total;
        storage_used = used;
        free_space_kb = (total - used) / 1024;
        ESP_LOGI("STORAGE", "Free space refreshed: %lu KB", free_space_kb);
    } else {
        ESP_LOGE("STORAGE", "Failed to get LittleFS info (%s)", esp_err_to_name(ret));
    }
}

// LEDS POV
uint8_t g_poi_speed = 128; // Default speed

volatile bool g_reloading_pattern = false;

//default pattern
void default_rainbow_pattern() {
    for (int i = 0; i < MAX_LEDS; i++) {
        uint8_t r = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1));
        uint8_t g = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1 + 2));
        uint8_t b = (uint8_t)(127 + 127 * sin((i + frame_counter) * 0.1 + 4));

        float scale = g_brightness / 255.0;
        led_strip_set_pixel(led_strip, i, r * scale, g * scale, b * scale);
    }

    led_strip_refresh(led_strip);
    frame_counter++;
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Fire Animation
void fire_animation(led_strip_handle_t strip, uint8_t brightness) {
    static uint8_t heat[MAX_LEDS];
    static bool initialized = false;

    if (!initialized) {
        memset(heat, 0, sizeof(heat));
        initialized = true;
    }

    // Cool down every cell a little
    for(int i = 0; i < MAX_LEDS; i++) {
        heat[i] = qsub8(heat[i], esp_random() % ((255 / MAX_LEDS) * 2) + 1);
    }

    // Heat up the bottom a bit
    for(int i = 0; i < esp_random() % (MAX_LEDS / 5) + 1; i++) {
        heat[esp_random() % 3] = heat[esp_random() % 3] + (esp_random() % 160) + 96;
    }

    // Move heat up the strip
    for(int k = MAX_LEDS - 1; k >= 2; k--) {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // Convert heat to LED colors
    float scale = brightness / 255.0f;
    for(int j = 0; j < MAX_LEDS; j++) {
        uint8_t t192 = scale8(heat[j], 191);
        uint8_t r = t192;
        uint8_t g = scale8(t192, 100); // Less green than red
        uint8_t b = 0; // No blue

        // Adjust for overall brightness
        led_strip_set_pixel(strip, j, (uint8_t)(r * scale), (uint8_t)(g * scale), (uint8_t)(b * scale));
    }
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Ocean Waves Animation
void ocean_waves_animation(led_strip_handle_t strip, uint8_t brightness) {
    static uint32_t hue_offset = 0;
    float scale = brightness / 255.0f;

    for (int i = 0; i < MAX_LEDS; i++) {
        // Calculate a sine wave for position, offset by time
        uint8_t wave = (uint8_t)(127 + 127 * sin((i * 0.3 + hue_offset * 0.05)));

        // Blend between deep blue and turquoise
        uint8_t r = (uint8_t)(0 * scale);
        uint8_t g = (uint8_t)(scale8(wave, 150) * scale); // More green for turquoise
        uint8_t b = (uint8_t)(scale8(wave, 255) * scale); // Full blue for deep water

        led_strip_set_pixel(strip, i, r, g, b);
    }
    led_strip_refresh(strip);
    hue_offset++;
    vTaskDelay(pdMS_TO_TICKS(30));
}

// Custom Animation (Matrix Rain - simplified)
void custom_animation(led_strip_handle_t strip, uint8_t brightness) {
    static uint8_t trail[MAX_LEDS]; // Stores brightness of each LED
    static bool initialized = false;

    if (!initialized) {
        memset(trail, 0, sizeof(trail));
        initialized = true;
    }

    // Shift pixels down and fade
    for (int i = MAX_LEDS - 1; i > 0; i--) {
        trail[i] = trail[i-1];
        if (trail[i] > 0) { // Fade out
            trail[i] = qsub8(trail[i], 10);
        }
    }
    trail[0] = 0; // Clear the first pixel

    // Randomly start a new "drop" at the top
    if (esp_random() % 10 < 3) { // 30% chance to start a new drop
        trail[0] = (esp_random() % 100) + 155; // Bright green at the top
    }

    float scale = brightness / 255.0f;
    for (int i = 0; i < MAX_LEDS; i++) {
        uint8_t val = trail[i];
        if (val > 0) {
            // Green "rain" effect
            led_strip_set_pixel(strip, i, (uint8_t)(0 * scale), (uint8_t)(val * scale), (uint8_t)(0 * scale));
        } else {
            led_strip_set_pixel(strip, i, 0, 0, 0); // Off
        }
    }
    led_strip_refresh(strip);
    vTaskDelay(pdMS_TO_TICKS(50));
}

//flash animation
void run_flash_animation(uint32_t color) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    for (int i = 0; i < MAX_LEDS; i++) {
        led_strip_set_pixel(led_strip, i, r, g, b);
    }
    led_strip_refresh(led_strip);
    
}

void render_frame(uint8_t *frame_ptr) {
    if (frame_ptr == NULL) return;

    for (int j = 0; j < MAX_LEDS; j++) {
        int offset = j * 3;
        // Your specific mapping logic
        led_strip_set_pixel(led_strip, (MAX_LEDS - 1) - j, 
                            frame_ptr[offset],     
                            frame_ptr[offset + 1], 
                            frame_ptr[offset + 2]);
    }
    led_strip_refresh(led_strip);
}

// LEDS POV render task
void pov_render_task(void *pvParameters) {
    // 1. Buffers: Use 'static' to keep them off the stack
    static uint8_t read_buf[1024];
    static FILE* f = NULL;
    

    while (1) {
        // --- BUTTON STATE MACHINE ---
        uint32_t now = millis();
        bool pressed = g_btn_is_down;

        if (pressed) {
            if (g_button_state == BS_INITIAL) {
                g_down_time = now;
                g_button_state = BS_CLICK_DOWN;
                g_display_state = DS_WAITING;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK_DOWN");
            } else if (g_button_state == BS_CLICK_UP) {
                g_down_time = now;
                g_button_state = BS_CLICK2_DOWN;
                g_display_state = DS_WAITING2;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK2_DOWN");
            } else if (g_button_state == BS_CLICK2_UP) {
                g_down_time = now;
                g_button_state = BS_CLICK3_DOWN;
                g_display_state = DS_WAITING3;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK3_DOWN");
            } else if (g_button_state == BS_CLICK3_UP) {
                g_down_time = now;
                g_button_state = BS_CLICK4_DOWN;
                g_display_state = DS_WAITING4;
                g_display_state_last_updated = now;
            } else if (g_button_state == BS_CLICK4_UP) {
                g_down_time = now;
                g_button_state = BS_CLICK5_DOWN;
                g_display_state = DS_WAITING5;
                g_display_state_last_updated = now;
            } else if (g_button_state == BS_CLICK_DOWN && (now - g_down_time) >= 500) {
                g_button_state = BS_CLICK_HOLD;
                g_display_state = DS_VOLTAGE;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK_HOLD");
            } else if (g_button_state == BS_CLICK_HOLD && (now - g_down_time) >= 2000) {
                g_button_state = BS_CLICK_HOLD_LONG;
                g_display_state = DS_SHUTDOWN;
                g_display_state_last_updated = now;
                g_shut_down_at = now;
                ESP_LOGI("BTN", "CLICK_HOLD_LONG (SHUTDOWN)");
            } else if (g_button_state == BS_CLICK2_DOWN && (now - g_down_time) >= 500) {
                g_button_state = BS_CLICK2_HOLD;
                g_display_state = DS_BANK;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK2_HOLD (BANK)");
            } else if (g_button_state == BS_CLICK3_DOWN && (now - g_down_time) >= 500) {
                g_button_state = BS_CLICK3_HOLD;
                g_display_state = DS_BRIGHTNESS;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK3_HOLD (BRIGHTNESS)");
            } else if (g_button_state == BS_CLICK4_DOWN && (now - g_down_time) >= 500) {
                g_button_state = BS_CLICK4_HOLD;
                g_display_state = DS_SPEED;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "CLICK4_HOLD (SPEED)");
            } else if (g_button_state == BS_CLICK5_DOWN && (now - g_down_time) >= 500) {
                g_button_state = BS_CLICK5_HOLD;
                g_display_state = DS_PATTERN;
                g_display_state_last_updated = now;
            }
        } else {
            // Release logic
            if (g_button_state == BS_CLICK_DOWN) {
                g_button_state = BS_CLICK_UP;
                ESP_LOGI("BTN", "CLICK_UP");
            } else if (g_button_state == BS_CLICK2_DOWN) {
                g_button_state = BS_CLICK2_UP;
                ESP_LOGI("BTN", "CLICK2_UP");
            } else if (g_button_state == BS_CLICK3_DOWN) {
                g_button_state = BS_CLICK3_UP;
            } else if (g_button_state == BS_CLICK4_DOWN) {
                g_button_state = BS_CLICK4_UP;
            } else if (g_button_state == BS_CLICK5_DOWN) {
                g_button_state = BS_CLICK5_UP;
            } else if (g_button_state == BS_CLICK_HOLD) {
                g_button_state = BS_INITIAL;
                g_display_state = DS_PATTERN;
                g_display_state_last_updated = now;
            } else if (g_button_state == BS_CLICK2_HOLD) {
                int selection = ((now - g_down_time - 500) % 3500) / 500;
                if (selection == 0) g_current_bank = 0;
                else if (selection == 1) g_current_bank = 1;
                else if (selection == 2) g_current_bank = 2;
                else if (selection == 3) { g_shuffle_all_banks = true; g_shuffle_slots_only = false; }
                else {
                    g_current_bank = selection - 4;
                    g_shuffle_slots_only = true;
                    g_shuffle_all_banks = false;
                }
                g_reloading_pattern = true;
                g_button_state = BS_INITIAL;
                g_display_state = DS_PATTERN;
                ESP_LOGI("BTN", "BANK SELECTED: %d", g_current_bank);
            } else if (g_button_state == BS_CLICK3_HOLD) {
                uint32_t duration = now - g_down_time;
                if (duration < 1000) g_brightness = g_brightness_presets[0];
                else if (duration < 1500) g_brightness = g_brightness_presets[1];
                else if (duration < 2000) g_brightness = g_brightness_presets[2];
                else if (duration < 2500) g_brightness = g_brightness_presets[3];
                else if (duration < 3000) g_brightness = g_brightness_presets[4];
                else g_brightness = g_brightness_presets[5];
                
                g_button_state = BS_INITIAL;
                g_display_state = DS_PATTERN;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "BRIGHTNESS SELECTED: %d", g_brightness);
            } else if (g_button_state == BS_CLICK4_HOLD) {
                uint32_t duration = now - g_down_time;
                if (duration < 1000) g_selected_speed_index = 0;
                else if (duration < 1500) g_selected_speed_index = 1;
                else if (duration < 2000) g_selected_speed_index = 2;
                else if (duration < 2500) g_selected_speed_index = 3;
                else if (duration < 3000) g_selected_speed_index = 4;
                else g_selected_speed_index = 5;

                g_button_state = BS_INITIAL;
                g_display_state = DS_PATTERN;
                g_display_state_last_updated = now;
                ESP_LOGI("BTN", "SPEED SELECTED: %d", g_selected_speed_index);
            } else if (g_button_state == BS_CLICK5_HOLD) {
                g_button_state = BS_INITIAL;
            }
        }

        // Multi-click timeouts
        if (g_button_state == BS_CLICK_UP && (now - g_down_time) >= 500) {
            g_current_slot = (g_current_slot + 1) % 5;
            g_reloading_pattern = true;
            g_display_state = DS_PATTERN;
            g_button_state = BS_INITIAL;
            ESP_LOGI("BTN", "SINGLE CLICK -> SLOT %d", g_current_slot);
        } else if (g_button_state == BS_CLICK2_UP && (now - g_down_time) >= 500) {
            g_button_state = BS_INITIAL;
        } else if (g_button_state == BS_CLICK3_UP && (now - g_down_time) >= 500) {
            g_button_state = BS_INITIAL;
        } else if (g_button_state == BS_CLICK4_UP && (now - g_down_time) >= 500) {
            g_button_state = BS_INITIAL;
        } else if (g_button_state == BS_CLICK5_UP && (now - g_down_time) >= 500) {
            g_display_state = DS_VOLTAGE2;
            g_display_state_last_updated = now;
            g_button_state = BS_INITIAL;
        }

        // Shutdown handling
        if (g_shut_down_at != 0 && (now - g_shut_down_at) > 2000) {
            led_strip_clear(led_strip);
            led_strip_refresh(led_strip);
            gpio_set_level(REGULATOR_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW);
            esp_deep_sleep_start();
        }

        // --- DISPLAY FEEDBACK ---
        if (g_display_state != DS_PATTERN) {
            if (g_display_state == DS_VOLTAGE || g_display_state == DS_VOLTAGE2) {
                float voltage = read_battery_voltage();
                uint8_t r = 0, g = 0, b = 0;
                if (voltage > 3.8f) g = 50;
                else if (voltage > 3.6f) { r = 50; g = 50; }
                else r = 50;
                
                int leds = (int)((voltage - 3.4f) * (MAX_LEDS / (4.2f - 3.4f)));
                if (leds < 1) leds = 1;
                if (leds > MAX_LEDS) leds = MAX_LEDS;
                
                led_strip_clear(led_strip);
                for(int i=0; i<leds; i++) led_strip_set_pixel(led_strip, i, r, g, b);
                led_strip_refresh(led_strip);
            } else if (g_display_state == DS_BANK) {
                int selection = ((now - g_down_time - 500) % 3500) / 500;
                uint8_t r=0, g=0, b=0;
                if (selection == 0) r = 50;
                else if (selection == 1) g = 50;
                else if (selection == 2) b = 50;
                else if (selection == 3) { r=30; g=30; b=30; }
                else { r=50; g=20; }
                
                led_strip_clear(led_strip);
                for(int i=0; i<MAX_LEDS; i++) led_strip_set_pixel(led_strip, i, r, g, b);
                led_strip_refresh(led_strip);
            } else if (g_display_state == DS_BRIGHTNESS) {
                uint32_t duration = now - g_down_time;
                int level = 1;
                if (duration > 3000) level = 6;
                else if (duration > 2500) level = 5;
                else if (duration > 2000) level = 4;
                else if (duration > 1500) level = 3;
                else if (duration > 1000) level = 2;
                
                led_strip_clear(led_strip);
                for(int i=0; i<(level * MAX_LEDS / 6); i++) led_strip_set_pixel(led_strip, i, 40, 40, 0);
                led_strip_refresh(led_strip);
            } else if (g_display_state == DS_SPEED) {
                uint32_t duration = now - g_down_time;
                int level = 1;
                if (duration > 3000) level = 6;
                else if (duration > 2500) level = 5;
                else if (duration > 2000) level = 4;
                else if (duration > 1500) level = 3;
                else if (duration > 1000) level = 2;
                
                led_strip_clear(led_strip);
                for(int i=0; i<(level * MAX_LEDS / 6); i++) led_strip_set_pixel(led_strip, i, 0, 40, 40);
                led_strip_refresh(led_strip);
            } else if (g_display_state == DS_SHUTDOWN) {
                run_flash_animation(0xFF0000);
                vTaskDelay(pdMS_TO_TICKS(50));
                run_flash_animation(0x000000);
            } else if (g_display_state >= DS_WAITING && g_display_state <= DS_WAITING5) {
                int count = (int)g_display_state - (int)DS_WAITING + 1;
                led_strip_clear(led_strip);
                for(int i=0; i<count; i++) led_strip_set_pixel(led_strip, i, 20, 20, 20);
                led_strip_refresh(led_strip);
            }

            if (now - g_display_state_last_updated > 2000 && !pressed) {
                g_display_state = DS_PATTERN;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // --- B. STATE SWITCHING ---
if ((xTaskGetTickCount() - g_last_battery_check) > pdMS_TO_TICKS(10000)) {    g_battery_voltage = read_battery_voltage();
 
    if (g_battery_voltage < 3.45f) {
       while (1) {
        g_emergency_mode = true;
        led_task_running = false; // Stop normal patterns
        show_sos_signal();
        vTaskDelay(pdMS_TO_TICKS(1000));
	     }
    } 
    
        if (g_battery_voltage < 3.40f) {
        led_task_running = false; // Stop normal patterns
        show_sos_signal();
        gpio_set_level(REGULATOR_GPIO, 0);
		esp_deep_sleep_enable_gpio_wakeup(1ULL << BUTTON_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW);
		esp_deep_sleep_start();	     
    } 
    g_last_battery_check = xTaskGetTickCount();
} 

if (current_mode == MODE_STREAMING) {
    // Wait for the timer to tell us it's time for the next frame
    // We wait up to 100ms, but the timer will usually wake us much faster
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
        if (xSemaphoreTake(frames_sem, 0) == pdTRUE) {
            render_frame(ring_buf->frames[ring_buf->tail]);
            
            // Update pointers
            ring_buf->tail = (ring_buf->tail + 1) % MAX_FRAMES;
            ring_buf->total_played++;
        }
    }
    continue; 
}
        
// --- Handle Animations if no file is loaded, or a specific animation is selected ---
if (f == NULL) {
    switch (g_current_animation) {
        case 0: // Default Rainbow Pattern
            default_rainbow_pattern();
            break;
        case 1: // Fire Animation
            fire_animation(led_strip, g_brightness);
            break;
        case 2: // Ocean Waves Animation
            ocean_waves_animation(led_strip, g_brightness);
            break;
        case 3: // Custom Animation (Matrix Rain)
            custom_animation(led_strip, g_brightness);
            break;
        default:
            default_rainbow_pattern(); // Fallback
            break;
    }
    continue; // Continue to the next loop iteration after rendering an animation
}

        // --- SHUFFLE TIMER CHECK ---
        if (g_shuffle_slots_only || g_shuffle_all_banks) {
            uint32_t now = xTaskGetTickCount();
            if (pdTICKS_TO_MS(now - g_last_shuffle_tick) > g_shuffle_duration_ms) {

                if (g_shuffle_all_banks) {
                    // Pick a random bank (assuming 0-3) and random slot (0-7)
                    g_current_bank = esp_random() % 3;
                    g_current_slot = esp_random() % 4;
                } else {
                    // Shuffle only slots in current bank
                    g_current_slot = esp_random() % 4;
                }

                g_reloading_pattern = true; // Trigger the file-opener logic
                g_last_shuffle_tick = now;
            }
        }

        // --- FILE OPENING ---

        if (g_reloading_pattern || f == NULL) {
            if (f) { fclose(f); f = NULL; }
            char path[32];
            // Using modulo to enforce Bank 0-2 and Slot 0-4 constraints
            snprintf(path, sizeof(path), "/littlefs/b%d_s%d.bin", (g_current_bank % 3), (g_current_slot % 5));
            
            f = fopen(path, "rb");
            if (f) {
                uint8_t header[3];
                if (fread(header, 1, 3, f) == 3) {
                    file_h = header[0]; // Raw height from file (e.g., 10 or 24)
                    img_w = (header[1] << 8) | header[2];
                    
                    // Reset buffer state for the new file
                    bytes_in_buf = 0; 
                    buf_pos = 0;
                    // Header is already read, pointer is at byte 3 (start of pixels)
                } else { 
                    fclose(f); 
                    f = NULL; 
                }
            }
            g_reloading_pattern = false;
        }

        if (f == NULL) {
            default_rainbow_pattern();
            vTaskDelay(pdMS_TO_TICKS(10)); // Prevent tight-looping
            continue;
        }

        // --- THE BUFFER & MODULO RENDER LOGIC ---

int slice_size = file_h * 3;

// 2. Refill the buffer if we don't have enough for a full file slice
if (buf_pos + slice_size > bytes_in_buf) {
    uint16_t remaining = bytes_in_buf - buf_pos;
    if (remaining > 0) memmove(read_buf, &read_buf[buf_pos], remaining);
    bytes_in_buf = remaining;
    buf_pos = 0;

    size_t n = fread(&read_buf[bytes_in_buf], 1, 1024 - bytes_in_buf, f);
    if (n == 0) { // End of file: loop back to start (after 3-byte header)
        fseek(f, 3, SEEK_SET);
        n = fread(&read_buf[bytes_in_buf], 1, 1024 - bytes_in_buf, f);
    }
    bytes_in_buf += n;
}

// If, after attempting to refill, we STILL don't have enough data 
        // (e.g., partial read, tiny file, or split packet), skip rendering this frame.
        if (bytes_in_buf < buf_pos + slice_size) {
            vTaskDelay(1); // Yield to let system catch up
            continue;
        }
        
if (bytes_in_buf >= slice_size) {
        // 3. FINAL DISPLAY (Modulo Scaling)
        uint8_t *slice = &read_buf[buf_pos];
        float scale = g_brightness / 255.0f;

        // 1. Loop through the PHYSICAL LEDs (20)
        for (int j = 0; j < MAX_LEDS; j++) {

    int data_row = j % file_h; 
    int offset = data_row * 3;

    led_strip_set_pixel(led_strip, (MAX_LEDS - 1) - j, 
                        (uint8_t)(slice[offset] * scale), 
                        (uint8_t)(slice[offset+1] * scale), 
                        (uint8_t)(slice[offset+2] * scale));
}

led_strip_refresh(led_strip);

        // Advance buf_pos by the ACTUAL FILE slice size
        buf_pos += (slice_size);
}
        // 4. SPEED HANDLING
        uint8_t current_raw_speed = g_speed_presets[g_selected_speed_index];
        int delay_ms = ((255 - current_raw_speed) / 5) + 1;

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}
//END of render task


// advertising cleanup
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        conn_hdl = (event->connect.status == 0) ? event->connect.conn_handle : 0xFFFF;
        ESP_LOGI("BLE", "Connected! Status: %d", event->connect.status);
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW("BLE", "Disconnected! Reason: %d. Restarting...", event->disconnect.reason);
        conn_hdl = 0xFFFF;
        // Safety: Reset multipart flag so we don't stay in "Loading" state
        g_multipart_active = false;

        // AUTO-RECOVERY: Start advertising, stop the pov_timer and enter the render loop again
        led_task_running = true;
        esp_timer_stop(pov_timer);
        current_mode = MODE_PATTERN;
        while(xSemaphoreTake(frames_sem, 0));
        start_advertising();

        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI("BLE", "MTU update: %d", event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI("BLE", "Connection updated");
        return 0;

    default:
        return 0;
    }
    return 0;
}

// --- Advertising Logic ---
void start_advertising(void) {
    struct ble_hs_adv_fields fields;
    int rc;

    // --- 1. Main Advertising Packet ---
    memset(&fields, 0, sizeof(fields));
    const char *name = "Open Pixel Poi";
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE("BLE", "Error setting main adv fields; rc=%d", rc);
        return;
    }

    // --- 2. Scan Response Packet ---
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    rsp_fields.uuids128 = (ble_uuid128_t[]){
        BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E)
    };
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE("BLE", "Error setting scan response fields; rc=%d", rc);
        return;
    }

    // --- 3. Advertising Parameters ---
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // --- FIXED: REGISTER THE HANDLER HERE ---
    // We replace NULL with ble_gap_event_handler
    rc = ble_gap_adv_start(BLE_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_handler, NULL);

    if (rc != 0) {
        ESP_LOGE("BLE", "Error starting advertisement; rc=%d", rc);
    } else {
        ESP_LOGI("BLE", "Advertising successfully started with Event Handler!");
    }
}

// --- Sync Callback: Triggers when BLE is actually ready ---
static void on_sync(void) {
    uint8_t addr_type;
    // Determine address type automatically
    ESP_ERROR_CHECK(ble_hs_id_infer_auto(0, &addr_type));
    start_advertising();
}

// Define the success response: [Start, SuccessCode, End]
// CC_SUCCESS is usually 0 in most enums, check your CommCode.dart

typedef enum {
    CC_SUCCESS = 0,
    CC_ERROR = 1,
    CC_SET_BRIGHTNESS = 2,
    CC_SET_SPEED = 3,
    CC_SET_PATTERN = 4,
    CC_SET_PATTERN_SLOT = 5,
    CC_SET_PATTERN_ALL = 6,
    CC_SET_BANK = 7,
    CC_SET_BANK_ALL = 8,
    CC_GET_FW_VERSION = 9,
    CC_SET_HARDWARE_VERSION = 10,
    CC_SET_LED_TYPE = 11,
    CC_SET_LED_COUNT = 12,
    CC_SET_DEVICE_NAME = 13,
    CC_SET_SEQUENCER = 14,
    CC_START_SEQUENCER = 15,
    CC_SET_BRIGHTNESS_OPTION = 16,
    CC_SET_BRIGHTNESS_OPTIONS = 17,
    CC_SET_SPEED_OPTION = 18,
    CC_SET_SPEED_OPTIONS = 19,
    CC_SET_PATTERN_SHUFFLE_DURATION = 20,
    CC_START_STREAM = 21,
    CC_STOP_STREAM = 22,
    CC_GET_CONFIG = 23,
    CC_STREAM_DATA = 24,
    CC_SET_ANIMATION = 25
} poi_comm_code_t;

// multi part upload
// --- HELPER FUNCTIONS (Place these above gatt_svr_cb) ---
void storage_worker_task(void *pvParameters) {
    flash_packet_t pkt;
    FILE* f = NULL;
    static char active_path[32];

    while (1) {
        if (xQueueReceive(flash_queue, &pkt, portMAX_DELAY)) {
            if (f == NULL) {
                // Ensure slot/bank are valid per requirements
                snprintf(active_path, sizeof(active_path), "/littlefs/b%d_s%d.bin", 
                         (g_current_bank % 3), (g_current_slot % 5));
                
                f = fopen(active_path, "wb");
                if (!f) {
                    ESP_LOGE("STORAGE", "Failed to open %s for writing!", active_path);
                    g_multipart_active = false;
                    continue;
                }
            }

            // Calculate actual data length excluding the terminator
            int data_end = pkt.len;
            if (pkt.is_final) {
                for (int i = pkt.len - 1; i >= 0; i--) {
                    if (pkt.pkt_data[i] == 0xD1) { data_end = i; break; }
                }
            }

            int final_write_len = data_end - pkt.skip_bytes;
            if (final_write_len > 0 && f) {
                fwrite(pkt.pkt_data + pkt.skip_bytes, 1, final_write_len, f);
            }

            if (pkt.is_final && f) {
                fflush(f);
                fsync(fileno(f));
                fclose(f);
                f = NULL;
                
                g_multipart_active = false;
                led_task_running = true;
                g_reloading_pattern = true; // Refresh the LED task
                ESP_LOGI("STORAGE", "Successfully saved %s", active_path);
            }
        }
    }
}

//helper function
void setTxCharacteristicValue(uint8_t* data, uint16_t len) {
    if (len > sizeof(tx_buffer)) len = sizeof(tx_buffer);

    // Copy the actual bytes
    memcpy(tx_buffer, data, len);
    tx_len = len;

    // Log the actual hex bytes to the console so you can see them
    // BEFORE the phone reads them.
    ESP_LOG_BUFFER_HEX("BLE_TX_DEBUG", tx_buffer, tx_len);
}

// --- GATT Callback ---

static int gatt_svr_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    conn_hdl = conn_handle;
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (ble_resp_len > 0) {
            return os_mbuf_append(ctxt->om, ble_resp_buf, ble_resp_len);
        }
        return 0; // Or return an error code if you prefer
    }

    //some debug
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_DSC) {
        uint16_t dsc_val = 0;
        if (OS_MBUF_PKTLEN(ctxt->om) >= 2) {
            os_mbuf_copydata(ctxt->om, 0, 2, &dsc_val);
            if (dsc_val == 0x0001) {
                ESP_LOGW("BLE_DEBUG", ">>> Phone just ENABLED notifications!");
            } else if (dsc_val == 0x0000) {
                ESP_LOGW("BLE_DEBUG", ">>> Phone just DISABLED notifications!");
            }
        }
    }
    //debug end

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t data[512];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(data)) len = sizeof(data);
        os_mbuf_copydata(ctxt->om, 0, len, data);

        uint8_t *payload = &data[2];
        int payload_len = (len > 2) ? (len - 2) : 0;

        // --- COMMAND HANDLING (0xD0) ---
        if (len >= 2 && data[0] == 0xD0) {
            poi_comm_code_t code = (poi_comm_code_t)data[1];

            // Handle standard commands
		switch (code) {

			case CC_SET_PATTERN: {
				flash_packet_t pkt = {0};
				pkt.len = len;

				// 1. Identify Start of Pattern
				if (len >= 5 && data[0] == 0xD0 && data[1] == CC_SET_PATTERN) {
					pkt.skip_bytes = 2;       // Skip protocol header
					g_multipart_active = true;
					led_task_running = false; // Stop LEDs to prioritize Flash write
					ESP_LOGI("BLE", "Pattern Start Detected");
				} else {
					pkt.skip_bytes = 0;
				}

				// 2. CATCH SMALL UPLOADS (Single Packet Case)
				// Check if the 0xD1 end marker is present in this same packet
				if (len > 0 && data[len - 1] == 0xD1) {
					pkt.is_final = true;
					g_multipart_active = false; // No more packets coming
					ESP_LOGI("BLE", "Single-packet upload complete");
				} else {
					// If it's a large file, the first packet won't have 0xD1
					pkt.is_final = false; 
				}

				memcpy(pkt.pkt_data, data, len);
				xQueueSend(flash_queue, &pkt, portMAX_DELAY);
				break;
			}

			case CC_GET_CONFIG:
			{
				ESP_LOGI("BLE", "Config request received");

				// 1. Get current free storage space and battery voltage
				update_storage_stats();
				g_battery_voltage = read_battery_voltage();
				uint16_t battery_val = (uint16_t)(g_battery_voltage * 100.0f);

				// 2. Prepare the 10-byte response packet
				uint8_t config_resp[12];
				config_resp[0] = 0xD0;                                 //start byte
				config_resp[1] = CC_GET_CONFIG;                        //command
				config_resp[2] = (uint8_t)MAX_LEDS;				       // number of leds
				config_resp[3] = 0x01;                                 // Protocol Version      
				config_resp[4] = (uint8_t)((MAX_FRAMES >> 8) & 0xFF);  // frame buffer size
				config_resp[5] = (uint8_t)(MAX_FRAMES & 0xFF);         // frame buffer size
				config_resp[6] = (uint8_t)((200 >> 8) & 0xFF);         // Hardware buffer limit
				config_resp[7] = (uint8_t)(200 & 0xFF);                // Hardware buffer limit
				config_resp[8] = (uint8_t)(battery_val >> 8);          // Battery voltage
				config_resp[9] = (uint8_t)(battery_val & 0xFF);        // Battery voltage
				config_resp[10] = (uint8_t)((free_space_kb >> 8) & 0xFF); 
				config_resp[11] = (uint8_t)(free_space_kb & 0xFF);
				// 3. SET THE CHAR VALUE (Instead of Notifying)
				// This allows the browser to 'Read' the result immediately 
				// after sending the command.
				set_ble_reply(config_resp, sizeof(config_resp));
				setTxCharacteristicValue(config_resp, sizeof(config_resp));
				
				ESP_LOGI("BLE", "TX characteristic updated with config and battery: %d", battery_val);
			}
			break;

            case CC_SET_BRIGHTNESS:
                ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
                if (payload_len >= 1) g_brightness = payload[0];
                ESP_LOGI("BLE", "Brightness request");
                break;

            case CC_SET_BRIGHTNESS_OPTION:
                ESP_LOGI("BLE_BRIGHT", "Brightness option request");
                ESP_LOG_BUFFER_HEX("BLE_BRIGHT_RAW", data, len);

                // Using the same 4-byte structure: [Start][Cmd][Index][End]
                if (len >= 3) {
                    uint8_t index = data[2];

                    if (index < 6) {
                        g_selected_brightness_index = index;
                        // Update the actual brightness variable used in the LED loop
                        g_brightness = g_brightness_presets[g_selected_brightness_index];

                        ESP_LOGW("BLE_BRIGHT", ">>> Gear %d selected. Real Brightness: %d/255",
                                 index, g_brightness);
                    } else {
                        ESP_LOGE("BLE_BRIGHT", ">>> ERROR: Index %d out of bounds", index);
                    }
                }
                break;

            //stream case start stuff

			case CC_START_STREAM: {
				uint16_t requested_hz = 200; 
				
				// Fix: Your byte-shifting logic had a typo (data[2] used twice)
				if (len >= 3) {
					ESP_LOGI("BLE_DATA", "Start Stream Bytes: [2]:0x%02X, [3]:0x%02X", data[1], data[2]);
					requested_hz = (data[2] << 8) | data[3]; // Index 1 and 2 if header is [PREFIX, CMD]
					ESP_LOGI("BLE_DATA", "Calculated Frequency: %u Hz", requested_hz);
				}

				if (requested_hz > 0) {
					uint64_t period_us = 1000000 / requested_hz;

					esp_timer_stop(pov_timer);

					// Reset buffer pointers
					ring_buf->head = 0;
					ring_buf->tail = 0;
					while(xSemaphoreTake(frames_sem, 0));

					// Change mode - This tells the big task below to stop reading files
					current_mode = MODE_STREAMING; 

					esp_timer_start_periodic(pov_timer, period_us);
					ESP_LOGI("BLE", "Stream Started: %u Hz", requested_hz);
					
					// Notify the task to wake up immediately
					xTaskNotifyGive(pov_task_handle);
				}
			}
			break;

            case CC_STOP_STREAM: {
                // Halt the timer immediately
                esp_timer_stop(pov_timer);
                current_mode = MODE_PATTERN;
                
                // Clear playback state
                while(xSemaphoreTake(frames_sem, 0));
                current_frame_idx = 0;

                // Clear the physical LEDs so they don't stay "stuck" on
                
                g_reloading_pattern = true;
                ESP_LOGI("BLE", "Stream Stopped: Timer halted and LEDs cleared");
            }
            break;

			case CC_STREAM_DATA: {
				uint8_t temp_flat_buf[MAX_BLE_PAYLOAD];
				uint16_t actual_len;

				is_writing_to_buffer = true;
				led_task_running = false; 

				int rc = ble_hs_mbuf_to_flat(ctxt->om, temp_flat_buf, sizeof(temp_flat_buf), &actual_len);

				// FIX: We now account for a 2-byte header [PREFIX, COMMAND]
				const int HEADER_SIZE = 2; 

				if (rc == 0 && actual_len >= (HEADER_SIZE + FRAME_SIZE)) {
					
					// Calculate frames based on length minus the 2-byte header
					int frames_received = (actual_len - HEADER_SIZE) / FRAME_SIZE;

					for (int f = 0; f < frames_received; f++) {
						// src_offset: skip 2 header bytes, then skip previous frames
						int src_offset = HEADER_SIZE + (f * FRAME_SIZE);
						
						// Safety check to prevent buffer overflow
						if (src_offset + FRAME_SIZE <= actual_len) {
							uint16_t next_head = (ring_buf->head + 1) % MAX_FRAMES;
							if (next_head != ring_buf->tail) {
								memcpy(ring_buf->frames[ring_buf->head], &temp_flat_buf[src_offset], FRAME_SIZE);
								ring_buf->head = next_head;
								xSemaphoreGive(frames_sem);
							} else {
								ESP_LOGW("BLE", "Stream buffer full, dropping frame");
								break; 
							}
						}
					}
				} else {
					ESP_LOGE("BLE", "Stream error: len=%d (Need >= %d)", actual_len, HEADER_SIZE + FRAME_SIZE);
				}
				
				is_writing_to_buffer = false;
			}
			break;

			case CC_SET_ANIMATION:
				if (payload_len >= 1) {
					g_current_animation = payload[0];
					current_mode = MODE_PATTERN; // Switch to pattern mode to run procedural animations
					
					// Clear the LED strip immediately to show the change
					if (led_strip) {
						led_strip_clear(led_strip);
						led_strip_refresh(led_strip);
					}
					ESP_LOGI("BLE", "Animation set to: %d", g_current_animation);
					set_ble_reply(resp_success, sizeof(resp_success));
				} else {
					ESP_LOGE("BLE", "CC_SET_ANIMATION received without payload");
					set_ble_reply(resp_error, sizeof(resp_error));
				}
				break;

            case CC_SET_SPEED_OPTION:
                ESP_LOGI("BLE", "Speed option request");
                ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
                if (len >= 2) {
                    uint8_t index = data[2];
                    if (index < 6) {
                        g_selected_speed_index = index;
                        ESP_LOGI("BLE", "Switched to Speed Gear: %d (Value: %d)",
                                 g_selected_speed_index, g_speed_presets[g_selected_speed_index]);
                    }
                }
                break;

            case CC_SET_SPEED_OPTIONS:
                ESP_LOGI("BLE", "Speed bank request");
                // If the phone sends 6 bytes, we update the whole gearbox
                if (len >= 7) {
                    for (int i = 0; i < 6; i++) {
                        g_speed_presets[i] = data[i+1];
                    }
                    ESP_LOGI("BLE", "Speed Gearbox Reprogrammed!");
                }
                break;

            case CC_SET_SPEED:
                ESP_LOGI("BLE", "Set speed reqwuest received");
                break;

            case CC_SET_BANK:
                if (len >= 3) g_current_bank = data[2];
                break;

            case CC_SET_SEQUENCER:
                ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
                break;

            case CC_START_SEQUENCER:
                ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
                break;

			case CC_SET_PATTERN_SLOT:
				if (len >= 3) {
					// Enforce the 0x00 to 0x04 constraint mentioned in your saved info
					uint8_t requested_slot = data[2];
					if (requested_slot <= 0x04) {
						g_current_slot = requested_slot;
						
						// 1. Force the render task to close the old file and open the new one
						g_reloading_pattern = true; 
						
						// 2. Reset buffer pointers immediately
						// This prevents the render task from trying to read from the 
						// old 'read_buf' while the new file is being opened.
						bytes_in_buf = 0;
						buf_pos = 0;

						// 3. Clear the LED hardware immediately (Optional but recommended)
						// This stops the "frozen" frame of the old slot from showing.
						if (led_strip) {
							led_strip_clear(led_strip);
							led_strip_refresh(led_strip);
						}

						ESP_LOGI("BLE", "Slot changed to %d. Buffer cleared.", g_current_slot);
					}
				}
				break;


            case CC_GET_FW_VERSION:
                ESP_LOGI("BLE", "Firmware version requested, sending direct response...");
                //uint8_t response[] = {0xD0, 0x00, 0x06, 0x09, 0x02, 0xD1};

                set_ble_reply(resp_firmware, sizeof(resp_firmware));
                //setTxCharacteristicValue(response, 6);
                break;

            case CC_SET_PATTERN_ALL: // Shuffle all slots in CURRENT bank (d0 06 d1)
                ESP_LOGI("SHUFFLE", "Toggle Bank Shuffle Requested");

                // Toggle the state: If it was true, it becomes false. If false, true.
                g_shuffle_slots_only = !g_shuffle_slots_only;

                // Safety: If we turn this on, make sure the other shuffle mode is off
                if (g_shuffle_slots_only) {
                    g_shuffle_all_banks = false;
                    g_last_shuffle_tick = xTaskGetTickCount(); // Reset timer on start
                }

                ESP_LOGW("SHUFFLE", "Bank Shuffle is now: %s", g_shuffle_slots_only ? "ON" : "OFF");
                set_ble_reply(resp_success, sizeof(resp_success));
                break;

            case CC_SET_BANK_ALL: // Shuffle across ALL banks (d0 08 d1)
                ESP_LOGI("SHUFFLE", "Toggle Global Shuffle Requested");

                // Toggle the state
                g_shuffle_all_banks = !g_shuffle_all_banks;

                if (g_shuffle_all_banks) {
                    g_shuffle_slots_only = false;
                    g_last_shuffle_tick = xTaskGetTickCount(); // Reset timer on start
                }

                ESP_LOGW("SHUFFLE", "Global Shuffle is now: %s", g_shuffle_all_banks ? "ON" : "OFF");
                set_ble_reply(resp_success, sizeof(resp_success));
                break;

            case CC_SET_PATTERN_SHUFFLE_DURATION: // Set the timer (e.g., 0-255 seconds)
                ESP_LOG_BUFFER_HEX("BLE_SPEED_RAW", data, len);
                // If your app sends the value in seconds at data[2]
                g_shuffle_duration_ms = data[2] * 1000;
                if (g_shuffle_duration_ms < 1000) {
                    g_shuffle_duration_ms = 1000; // Min 1s
                    set_ble_reply(resp_success, sizeof(resp_success)); // Set 0x00 Success
                } else {
                    set_ble_reply(resp_error, sizeof(resp_error));     // Set 0x01 Error
                }
                break;

            default:
                break;
            }
            return 0;
        }

        // --- DATA CHUNK HANDLING (Multipart) ---
if (g_multipart_active) {
    flash_packet_t pkt = {0}; // Initialize to 0 so is_final is false by default
    pkt.len = len;
    pkt.skip_bytes = 0;
    memcpy(pkt.pkt_data, data, len);

    // Check if this specific chunk contains the end-of-file marker (0xD1)
    if (len > 0 && data[len - 1] == 0xD1) {
        pkt.is_final = true;
    } else {
        // Fallback: if the packet is smaller than a standard MTU chunk, 
        // it's likely the last one.
        pkt.is_final = (len < 500); 
    }

    if (xQueueSend(flash_queue, &pkt, pdMS_TO_TICKS(10)) != pdPASS) {
        ESP_LOGW("BLE", "Queue full! Dropping packet");
        return BLE_ATT_ERR_PREPARE_QUEUE_FULL;
    }
    return 0;
}
    }
    return 0;
}

/* 1. Define Characteristics Array First */
static const struct ble_gatt_chr_def gatt_characteristics[] = {
    {
        // RX Characteristic
        .uuid = (ble_uuid_t *)&rx_uuid,
        .access_cb = gatt_svr_cb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        // TX Characteristic
        .uuid = (ble_uuid_t *)&tx_uuid,
        .access_cb = gatt_svr_cb,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        // NOTIFY Characteristic
        .uuid = (ble_uuid_t *)&notify_uuid,
        .access_cb = gatt_svr_cb,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &notify_handle,
    },
    {0} // Terminator
};

/* 2. Define the Service Table pointing to the array above */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t *)&svc_uuid,
        .characteristics = gatt_characteristics, // Point to the named array
    },
    {0}, // Terminator
};

void host_task(void *param) {
    ESP_LOGI("BLE", "NimBLE Host Task Started");

    /* This function will not return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up when the loop finishes */
    nimble_port_freertos_deinit();
}
//BLE functions


//flexible timer
// This function can be called anytime to change speed
void update_timer_frequency(int hz) {
    if (hz < 1) hz = 1;
    current_period_us = 1000000 / hz;

    esp_timer_stop(pov_timer);
    esp_timer_start_periodic(pov_timer, current_period_us);

    ESP_LOGI("TIMER", "Frequency set to %d Hz (%lu us)", hz, current_period_us);
}

//streaming timer
void IRAM_ATTR pov_timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(pov_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void init_flexible_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &pov_timer_callback,
        .name = "pov_playback"
    };
    esp_timer_create(&timer_args, &pov_timer);
}

// MAIN
void app_main(void) {
    // 1. Minimum OS setup
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize REGULATOR_GPIO early
    gpio_set_direction(REGULATOR_GPIO, GPIO_MODE_OUTPUT);

    // Check if it's a deep sleep wakeup
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_GPIO) {
        uint64_t wakeup_gpio_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_gpio_mask & (1ULL << BUTTON_GPIO)) {
            ESP_LOGI(TAG, "Woke up from deep sleep by GPIO %d (button)", BUTTON_GPIO);
            // No need for rtc_gpio_deinit as esp_deep_sleep_enable_gpio_wakeup handles it.
            // Re-enable normal GPIO pull-up/down if needed after wakeup for normal operation
            gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
            gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ENABLE);
        }
        // Re-enable regulator after deep sleep wakeup
        gpio_set_level(REGULATOR_GPIO, 1);
    } else {
        ESP_LOGI(TAG, "Not a deep sleep wakeup or unknown cause: %d", wakeup_cause);
        // Ensure regulator is on for fresh boot
        gpio_set_level(REGULATOR_GPIO, 1);
    }

    //init fs
    init_littlefs();

    // 2. NIMBLE FIRST (Synchronous)
    void check_memory(const char* label) {
        multi_heap_info_t info;
        heap_caps_get_info(&info, MALLOC_CAP_8BIT);
        printf("--- Memory Check: %s ---\n", label);
        printf("Total Free: %zu bytes\n", info.total_free_bytes);
        printf("Largest Block: %zu bytes\n", info.largest_free_block); 
        printf("Minimum Ever Free: %zu bytes\n", info.minimum_free_bytes);
        printf("---------------------------\n");
    }
    check_memory("BEFORE_NIMBLE_INIT");
    // Do NOT initialize LED strip or SPIFFS before this line
    // 5. Start the NimBLE Host Task
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE("BLE", "nimble_port_init failed: %d", ret);
        return;
    }
    // 3. Configure Host
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svr_svcs);
    ble_gatts_add_svcs(gatt_svr_svcs);
    // 2. Create the task manually with HIGH priority (e.g., 10 or 15)
    // This ensures Bluetooth always wins over the LED math and Flash writes
    xTaskCreate(host_task, "nimble_host", 8192, NULL, 15, NULL);

    // init queue
    flash_queue = xQueueCreate(20, sizeof(flash_packet_t));
    xTaskCreate(storage_worker_task, "storage_task", 6144, NULL, 4, NULL);
    // --- Initialize LED Strip ---
led_strip_config_t strip_config = {
    .strip_gpio_num = LED_GPIO, // Your LED pin
    .max_leds = MAX_LEDS,
    .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    .led_model = LED_MODEL_WS2812,
};

    led_strip_rmt_config_t rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 10 * 1000 * 500, // 10MHz
    .mem_block_symbols = 128,          

    };

ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    ESP_LOGI(TAG, "LED Strip Initialized.");
    // --- Button setup ---
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&btn_conf);

    // 1. Install the service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    // --- Button setup end ---


adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
};
ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

// 2. Configure the Channel (GPIO 4 is Channel 4)
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_12, // Necessary for voltages > 1.1V
};
ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adcchan, &config));
   
   
   
   //init buffer
    ring_buf = heap_caps_calloc(1, sizeof(pov_stream_buf_t), MALLOC_CAP_DMA | MALLOC_CAP_8BIT);

if (ring_buf == NULL) {
    ESP_LOGE("MEM", "Failed to allocate DMA ring buffer!");
}

    // Create counting semaphore for stream frames
    frames_sem = xSemaphoreCreateCounting(MAX_FRAMES, 0);
    if (frames_sem == NULL) {
        ESP_LOGE("MEM", "Failed to create frames semaphore!");
    }

    // 7. Start the POV Rendering Task
    // We give it a priority of 5 (Higher than the idle task, but balanced for BLE)
    xTaskCreate(pov_render_task, "led_task", 10240, NULL, 10, &pov_task_handle);
    //init stream timer

    init_flexible_timer();

    ESP_LOGI(TAG, "System Ready. Waiting for BLE Sync...");

    // The main task can now just sit and watch the system or be deleted
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
