#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_flash.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
// #include "ssd1306.h"
#include "driver/uart.h"
#include <string.h>
#include "driver/spi_master.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include <sys/stat.h>
#include <dirent.h>

static const char *TAG = "MAIN";

// Forward declarations
static void display_gps_battery_safe(void);
esp_err_t sd_card_init(void);
esp_err_t sd_save_gps_log(float lat, float lon, float battery);

/* ---------------- CONFIG ---------------- */
#define OLED_I2C_PORT I2C_NUM_0
#define OLED_ADDR     0x3C

#define OLED_SDA      10
#define OLED_SCL      9
// #define OLED_RES      10

#define OLED_CMD      0x00
#define OLED_DATA     0x40

#define OLED_WIDTH  128
#define OLED_HEIGHT 64


/* SSD1315 column offset */
#define COL_OFFSET    0
/* ================= LOW LEVEL ================= */

static esp_err_t oled_i2c_write(uint8_t control, uint8_t *data, uint16_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(OLED_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void oled_cmd(uint8_t cmd)
{
    oled_i2c_write(OLED_CMD, &cmd, 1);
}

static void oled_data(uint8_t *data, uint16_t len)
{
    oled_i2c_write(OLED_DATA, data, len);
}

/* ================= CORE ================= */

static void oled_set_pos(uint8_t x, uint8_t page)
{
    oled_cmd(0xB0 + page);
    oled_cmd(((x + COL_OFFSET) >> 4) | 0x10);
    oled_cmd((x + COL_OFFSET) & 0x0F);
}

void oled_clear(void)
{
    uint8_t zero[128] = {0};

    for (uint8_t p = 0; p < 8; p++)
    {
        oled_set_pos(0, p);
        oled_data(zero, 128);
    }
}

/* ================= FONT (6x8) ================= */

static const uint8_t font6x8[96][6] = {
{0,0,0,0,0,0},{0,0,95,0,0,0},{0,7,0,7,0,0},{20,127,20,127,20,0},
{36,42,127,42,18,0},{35,19,8,100,98,0},{54,73,85,34,80,0},{0,5,3,0,0,0},
{0,28,34,65,0,0},{0,65,34,28,0,0},{20,8,62,8,20,0},{8,8,62,8,8,0},
{0,80,48,0,0,0},{8,8,8,8,8,0},{0,96,96,0,0,0},{32,16,8,4,2,0},
{62,81,73,69,62,0},{0,66,127,64,0,0},{66,97,81,73,70,0},{33,65,69,75,49,0},
{24,20,18,127,16,0},{39,69,69,69,57,0},{60,74,73,73,48,0},{1,113,9,5,3,0},
{54,73,73,73,54,0},{6,73,73,41,30,0},{0,54,54,0,0,0},{0,86,54,0,0,0},
{8,20,34,65,0,0},{20,20,20,20,20,0},{0,65,34,20,8,0},{2,1,81,9,6,0},
{50,73,121,65,62,0},{126,17,17,17,126,0},{127,73,73,73,54,0},{62,65,65,65,34,0},
{127,65,65,34,28,0},{127,73,73,73,65,0},{127,9,9,9,1,0},{62,65,73,73,122,0},
{127,8,8,8,127,0},{0,65,127,65,0,0},{32,64,65,63,1,0},{127,8,20,34,65,0},
{127,64,64,64,64,0},{127,2,12,2,127,0},{127,4,8,16,127,0},{62,65,65,65,62,0},
{127,9,9,9,6,0},{62,65,81,33,94,0},{127,9,25,41,70,0},{70,73,73,73,49,0},
{1,1,127,1,1,0},{63,64,64,64,63,0},{31,32,64,32,31,0},{63,64,56,64,63,0},
{99,20,8,20,99,0},{3,4,120,4,3,0},{97,81,73,69,67,0},{0,127,65,65,0,0},
{2,4,8,16,32,0},{0,65,65,127,0,0},{4,2,1,2,4,0},{64,64,64,64,64,0},
{0,1,2,4,0,0},{32,84,84,84,120,0},{127,72,68,68,56,0},{56,68,68,68,32,0},
{56,68,68,72,127,0},{56,84,84,84,24,0},{8,126,9,1,2,0},{24,164,164,164,124,0},
{127,8,4,4,120,0},{0,68,125,64,0,0},{64,128,128,122,0,0},{127,16,40,68,0,0},
{0,65,127,64,0,0},{124,4,120,4,120,0},{124,8,4,4,120,0},{56,68,68,68,56,0},
{252,24,36,36,24,0},{24,36,36,24,252,0},{124,8,4,4,8,0},{72,84,84,84,36,0},
{4,63,68,64,32,0},{60,64,64,32,124,0},{28,32,64,32,28,0},{60,64,48,64,60,0},
{68,40,16,40,68,0},{12,144,144,144,124,0},{68,100,84,76,68,0},{0,8,54,65,0,0},
{0,0,127,0,0,0},{0,65,54,8,0,0},{16,8,8,16,8,0},{0,0,0,0,0,0}
};

/* ================= TEXT ================= */

static void oled_char(uint8_t x, uint8_t page, char c)
{
    if (c < 32 || c > 127) c = 32;
    oled_set_pos(x, page);
    oled_data((uint8_t *)font6x8[c - 32], 6);
}

void oled_show_string(uint8_t x, uint8_t y, const char *str)
{
    while (*str)
    {
        oled_char(x, y, *str++);
        x += 6;
        if (x > 122) break;
    }
}


/* ================= INIT ================= */

void oled_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA,
        .scl_io_num = OLED_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };

    esp_err_t ret;

    ret = i2c_param_config(OLED_I2C_PORT, &conf);
    ESP_LOGI(TAG, "i2c_param_config = %s", esp_err_to_name(ret));

    ret = i2c_driver_install(OLED_I2C_PORT, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "i2c_driver_install = %s", esp_err_to_name(ret));


    oled_cmd(0xAE);
    oled_cmd(0x20); oled_cmd(0x00);
    oled_cmd(0xA1);
    oled_cmd(0xC8);
    oled_cmd(0xA6);
    oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0xD5); oled_cmd(0x80);
    oled_cmd(0xD9); oled_cmd(0xF1);
    oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0xDB); oled_cmd(0x40);
    oled_cmd(0x8D); oled_cmd(0x14);
    oled_cmd(0xAF);

    oled_clear();
    ESP_LOGI(TAG, "SSD1315 init OK");
}

static bool uart_ready = false;

// Global GPS and battery variables for OLED display
static float g_gps_lat = 0.0f;
static float g_gps_lon = 0.0f;
static float g_battery_v = 0.0f;

// Previous values for differential display updates
static float g_gps_lat_prev = -999.0f;
static float g_gps_lon_prev = -999.0f;
static float g_battery_v_prev = -1.0f;

// OLED update control - prevent flickering
static SemaphoreHandle_t oled_mutex = NULL;

// SD Card globals
static sdmmc_card_t *sd_card = NULL;
const char *base_path = "/sd";

// SD Card SPI pins - adjust these to your actual pinout
#define SD_MOSI         11    // Shared with LoRa / or use different pins
#define SD_MISO         13    // Shared with LoRa / or use different pins  
#define SD_SCK          12    // Shared with LoRa / or use different pins
#define SD_CS           8     // SD Card chip select (unique, not shared)
#define SD_HOST         SPI3_HOST

#define UART_PORT_NUM       UART_NUM_0   
#define UART_TX_PIN         43
#define UART_RX_PIN         44
#define UART_BAUD_RATE      9600
#define BUF_SIZE            1024

static const int baud_rates[] = {9600, 4800, 38400, 19200, 115200};
#define NUM_BAUDS (sizeof(baud_rates)/sizeof(baud_rates[0]))

// Define the pins
#define RF_EN 39
#define GPS_EN 40

// RBG LED
#define LED_STRIP_USE_DMA  0
// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 1
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically
// GPIO assignment
#define LED_STRIP_GPIO_PIN  38
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

// Initialize ADC
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;

// Pin configuration
#define LORA_MOSI   11
#define LORA_MISO   13
#define LORA_SCK    12
#define LORA_CS     21
#define LORA_RST    14
#define LORA_DIO0   47

#define SPI_HOST    SPI2_HOST

spi_device_handle_t lora_spi;
/* ================= SX1262 COMMANDS ================= */

#define SX126X_GET_STATUS   0xC0
#define SX126X_SET_STANDBY  0x80

/* ================= SPI ================= */

static void sx126x_spi(uint8_t *tx, uint8_t *rx, int len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    ESP_ERROR_CHECK(spi_device_transmit(lora_spi, &t));
}

/* ================= SX1262 ================= */

void lora_reset(void)
{
    gpio_set_level(LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(LORA_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

uint8_t sx126x_get_status(void)
{
    uint8_t tx[2] = { SX126X_GET_STATUS, 0x00 };
    uint8_t rx[2] = { 0 };
    sx126x_spi(tx, rx, 2);
    return rx[1];
}

void sx126x_set_standby(void)
{
    uint8_t tx[2] = { SX126X_SET_STANDBY, 0x00 };   // RC standby
    uint8_t rx[2];
    sx126x_spi(tx, rx, 2);
}

// Initialize UART0
static void uart_init_once(int baud)
{
    uart_config_t uart_config = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM,
                                UART_TX_PIN,
                                UART_RX_PIN,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));
    uart_ready = true;
}

// Set UART0 baud dynamically
static void uart_set_baud(int baud)
{
    ESP_ERROR_CHECK(uart_set_baudrate(UART_PORT_NUM, baud));
    uart_flush(UART_PORT_NUM);
}

// Detect GPS baud automatically
static int detect_gps_baud(void)
{
    uint8_t data[BUF_SIZE];

    for (int i = 0; i < NUM_BAUDS; i++)
    {
        int baud = baud_rates[i];
        ESP_LOGI(TAG, "Trying GPS baud: %d", baud);

        uart_set_baud(baud);
        vTaskDelay(pdMS_TO_TICKS(1500));   // Wait for GPS data

        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));

        if (len > 0)
        {
            data[len] = 0;
            if (strstr((char *)data, "$GNRMC") || strstr((char *)data, "$GPGGA"))
            {
                ESP_LOGI(TAG, "GPS baud detected: %d", baud);
                return baud;
            }
        }
    }
    return -1;  // GPS not detected
}

// Convert NMEA lat/lon token (ddmm.mmmm or dddmm.mmmm) to decimal degrees
static float nmea_token_to_deg(const char *token)
{
    if (!token || !*token) return 0.0f;
    const char *dot = strchr(token, '.');
    if (!dot) return 0.0f;
    int int_len = (int)(dot - token);          // chars before '.'
    int deg_digits = int_len - 2;              // degrees digits = total int digits - 2 (minutes)
    if (deg_digits <= 0) return 0.0f;
    char deg_str[4] = {0};
    if (deg_digits >= (int)sizeof(deg_str)) return 0.0f;
    memcpy(deg_str, token, deg_digits);
    int deg = atoi(deg_str);
    float minutes = atof(token + deg_digits);
    return deg + minutes / 60.0f;
}

// Parse a single NMEA sentence and extract lat/lon if available.
// Returns true on success and fills *out_lat, *out_lon (decimal degrees).
static bool parse_nmea_get_latlon(char *nmea, float *out_lat, float *out_lon)
{
    if (!nmea) return false;

    // Strip NMEA checksum (everything after '*')
    char *checksum_pos = strchr(nmea, '*');
    if (checksum_pos) {
        *checksum_pos = '\0';  // terminate at '*'
    }

    ESP_LOGD(TAG, "[NMEA] Parsing: %.50s", nmea);

    // only handle GPRMC/GNRMC and GPGGA/GNGGA
    if (!(strncmp(nmea, "$GPRMC", 6) == 0 || strncmp(nmea, "$GNRMC", 6) == 0 ||
          strncmp(nmea, "$GPGGA", 6) == 0 || strncmp(nmea, "$GNGGA", 6) == 0)) {
        ESP_LOGD(TAG, "[NMEA] Unsupported sentence type: %.6s", nmea);
        return false;
    }

    // Tokenize safely (we modify the buffer)
    char *saveptr = NULL;
    char *tok = strtok_r(nmea, ",", &saveptr); // sentence id
    if (!tok) return false;

    // Check tok[4]: RMC has 'M' (like $GPRMC, $GNRMC)
    // GGA has 'A' (like $GPGGA, $GNGGA)
    if (tok[4] == 'M') {
        // RMC: $--RMC,time,status,lat,N/S,lon,E/W,...
        // tokens: 0=id,1=time,2=status,3=lat,4=N/S,5=lon,6=E/W
        char *time = strtok_r(NULL, ",", &saveptr); (void)time;
        char *status = strtok_r(NULL, ",", &saveptr);
        // Check status FIRST — if 'V', reject immediately (no fix)
        if (!status || status[0] == 'V') {
            return false;
        }
        char *lat_tok = strtok_r(NULL, ",", &saveptr);
        char *ns = strtok_r(NULL, ",", &saveptr);
        char *lon_tok = strtok_r(NULL, ",", &saveptr);
        char *ew = strtok_r(NULL, ",", &saveptr);
        if (!status || !lat_tok || !ns || !lon_tok || !ew) {
            return false;
        }
        if (status[0] != 'A') {
            return false;
        }
        float lat = nmea_token_to_deg(lat_tok);
        float lon = nmea_token_to_deg(lon_tok);
        if (ns[0] == 'S') lat = -lat;
        if (ew[0] == 'W') lon = -lon;
        *out_lat = lat;
        *out_lon = lon;
        return true;
    } else if (tok[4] == 'A') {
        // GGA: $--GGA,time,lat,N/S,lon,E/W,fix,...
        char *time = strtok_r(NULL, ",", &saveptr); (void)time;
        char *lat_tok = strtok_r(NULL, ",", &saveptr);
        char *ns = strtok_r(NULL, ",", &saveptr);
        char *lon_tok = strtok_r(NULL, ",", &saveptr);
        char *ew = strtok_r(NULL, ",", &saveptr);
        char *fix = strtok_r(NULL, ",", &saveptr);
        if (!lat_tok || !ns || !lon_tok || !ew || !fix) {
            return false;
        }
        int fix_int = atoi(fix);
        if (fix_int == 0) {
            return false;
        }
        float lat = nmea_token_to_deg(lat_tok);
        float lon = nmea_token_to_deg(lon_tok);
        if (ns[0] == 'S') lat = -lat;
        if (ew[0] == 'W') lon = -lon;
        *out_lat = lat;
        *out_lon = lon;
        return true;
    }
    return false;  // Unknown sentence type
}

// Replace your existing gps_task with this implementation.
// It accumulates bytes into lines and parses full NMEA sentences.
static void gps_task(void *arg)
{
    int baud = detect_gps_baud();
    if (baud < 0) {
        ESP_LOGE(TAG, "GPS not detected");
        vTaskDelete(NULL);
        return;
    }

    uint8_t data[BUF_SIZE];
    char linebuf[256];
    int line_idx = 0;
    float lat = 0.0f, lon = 0.0f;

    ESP_LOGI(TAG, "Starting GPS read at %d baud", baud);

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; ++i) {
            char c = (char)data[i];
            if (c == '\r') continue;
            if (c == '\n') {
                if (line_idx == 0) continue;
                linebuf[line_idx] = '\0';
                // Make a local copy for parsing since parse modifies buffer
                char tmp[256];
                strncpy(tmp, linebuf, sizeof(tmp) - 1);
                tmp[sizeof(tmp) - 1] = '\0';
                float parsed_lat, parsed_lon;
                if (parse_nmea_get_latlon(tmp, &parsed_lat, &parsed_lon)) {
                    lat = parsed_lat;
                    lon = parsed_lon;
                    g_gps_lat = lat;  // Update global for OLED display
                    g_gps_lon = lon;  // Update global for OLED display
                    ESP_LOGI(TAG, "Parsed GPS: lat=%.6f lon=%.6f", lat, lon);
                    display_gps_battery_safe();  // Update OLED with GPS and battery info (rate-limited)
                    // Log GPS data to SD card
                    if (sd_card) {
                        sd_save_gps_log(lat, lon, g_battery_v);
                    }
                } else {
                    ESP_LOGD(TAG, "NMEA (no fix or unsupported): %s", linebuf);
                }
                line_idx = 0;
                vTaskDelay(pdMS_TO_TICKS(1));  // Prevent watchdog timeout
            } else {
                if (line_idx < (int)sizeof(linebuf) - 1) {
                    linebuf[line_idx++] = c;
                } else {
                    // overflow: reset
                    line_idx = 0;
                }
            }
        }
    }
}


void battery_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,   // up to ~3.6V
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_cfg); // GPIO3

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_2,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali_handle);
}

float battery_get_voltage(void)
{
    int raw = 0;
    int mv = 0;

    adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &raw);
    adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv);

    // You have 100k / 100k divider → 2x
    return (mv * 2.0f) / 1000.0f;
}

// Display GPS coordinates and battery voltage on OLED with differential updates (only changed lines)
static void display_gps_battery_safe(void)
{
    if (!oled_mutex) return;

    // Check if ANY value changed - if not, skip update entirely
    if (g_gps_lat == g_gps_lat_prev && 
        g_gps_lon == g_gps_lon_prev && 
        g_battery_v == g_battery_v_prev) {
        return;  // No change detected, skip update
    }

    if (xSemaphoreTake(oled_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return;  // Could not acquire mutex, skip this update
    }

    char buf[40];
    
    // Update Line 0: Latitude (only if changed)
    if (g_gps_lat != g_gps_lat_prev) {
        oled_set_pos(0, 0);
        uint8_t clear_line[128] = {0};
        oled_data(clear_line, 128);  // Clear just this line
        snprintf(buf, sizeof(buf), "Lat: %+.4f", g_gps_lat);
        oled_show_string(0, 0, buf);
        g_gps_lat_prev = g_gps_lat;
    }
    
    // Update Line 1: Longitude (only if changed)
    if (g_gps_lon != g_gps_lon_prev) {
        oled_set_pos(0, 1);
        uint8_t clear_line[128] = {0};
        oled_data(clear_line, 128);  // Clear just this line
        snprintf(buf, sizeof(buf), "Lon: %+.4f", g_gps_lon);
        oled_show_string(0, 1, buf);
        g_gps_lon_prev = g_gps_lon;
    }
    
    // Update Line 2: Battery (only if changed)
    if (g_battery_v != g_battery_v_prev) {
        oled_set_pos(0, 2);
        uint8_t clear_line[128] = {0};
        oled_data(clear_line, 128);  // Clear just this line
        snprintf(buf, sizeof(buf), "Batt: %.2fV", g_battery_v);
        oled_show_string(0, 2, buf);
        g_battery_v_prev = g_battery_v;
    }

    xSemaphoreGive(oled_mutex);
}

/* ================= SD CARD ================= */

// Initialize SD card
esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card...");

    // Configure SPI bus for SD card (SPI3_HOST)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI,
        .miso_io_num = SD_MISO,
        .sclk_io_num = SD_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(SD_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Get default SDSPI host configuration
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_HOST;  // Set the host slot

    // Configure SD SPI device
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = SD_HOST;

    // Configure FATFS mount
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    ret = esp_vfs_fat_sdspi_mount(base_path, &host, &slot_config, &mount_config, &sd_card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem (formatting may be required)");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    // Print card info
    sdmmc_card_print_info(stdout, sd_card);
    ESP_LOGI(TAG, "SD card mounted successfully at %s", base_path);
    return ESP_OK;
}

// Deinitialize SD card
esp_err_t sd_card_deinit(void)
{
    if (sd_card) {
        esp_vfs_fat_sdcard_unmount(base_path, sd_card);
        sd_card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
    }
    return ESP_OK;
}

// Write string to file
esp_err_t sd_write_file(const char *filename, const char *data)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", base_path, filename);

    FILE *f = fopen(filepath, "w");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    fprintf(f, "%s", data);
    fclose(f);
    ESP_LOGI(TAG, "File written: %s", filepath);
    return ESP_OK;
}

// Append string to file
esp_err_t sd_append_file(const char *filename, const char *data)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", base_path, filename);

    FILE *f = fopen(filepath, "a");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for appending: %s", filepath);
        return ESP_FAIL;
    }

    fprintf(f, "%s", data);
    fclose(f);
    ESP_LOGI(TAG, "Data appended to file: %s", filepath);
    return ESP_OK;
}

// Read entire file into buffer
esp_err_t sd_read_file(const char *filename, char *buffer, size_t buffer_size)
{
    char filepath[512];
    snprintf(filepath, sizeof(filepath), "%s/%s", base_path, filename);

    FILE *f = fopen(filepath, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open file for reading: %s", filepath);
        return ESP_FAIL;
    }

    size_t read_size = fread(buffer, 1, buffer_size - 1, f);
    buffer[read_size] = '\0';
    fclose(f);

    ESP_LOGI(TAG, "File read: %s (%zu bytes)", filepath, read_size);
    return ESP_OK;
}

// Save GPS coordinates to file
esp_err_t sd_save_gps_log(float lat, float lon, float battery)
{
    char buffer[512];
    snprintf(buffer, sizeof(buffer), "Lat: %.6f, Lon: %.6f, Batt: %.2fV\n", lat, lon, battery);
    return sd_append_file("gps_log.txt", buffer);
}

// List files in SD card
esp_err_t sd_list_files(void)
{
    DIR *dir = opendir(base_path);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory: %s", base_path);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Files in %s:", base_path);
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            char filepath[512];
            snprintf(filepath, sizeof(filepath), "%s/%s", base_path, entry->d_name);
            struct stat file_stat;
            stat(filepath, &file_stat);
            ESP_LOGI(TAG, "  - %s (%ld bytes)", entry->d_name, file_stat.st_size);
        }
    }
    closedir(dir);
    return ESP_OK;
}

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

//OLED display handle
// static SSD1306_t dev;

// void init_oled(void) {
//     ESP_LOGI(TAG, "INTERFACE is i2c");
// 	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
// 	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
// 	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
// 	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
//     //Initialize the SSD1306 device
//     ssd1306_init(&dev, 128, 64); // width = 128, height = 64

//     //Clear the screen
//     ssd1306_clear_screen(&dev, false);

// 	ssd1306_contrast(&dev, 0xff);
// 	ssd1306_display_text(&dev, 0, "SSD1306 128x64", 14, false);
// 	ssd1306_display_text(&dev, 1, "ABCDEFGHIJKLMNOP", 16, false);
// 	ssd1306_display_text(&dev, 2, "abcdefghijklmnop",16, false);
// 	ssd1306_display_text(&dev, 3, "Hello World!!", 13, false);
// }


void app_main(void)
{

    esp_err_t ret;
    uint32_t flash_size;
    esp_chip_info_t chip_info;    

    gpio_config_t io_conf_en = {
        .pin_bit_mask = (1ULL << RF_EN) | (1ULL << GPS_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_en);

    // ENABLE RF AND GPS
    gpio_set_level(RF_EN, true);
    gpio_set_level(GPS_EN, true);

    oled_init();
    oled_show_string(0, 0, "MAKER");
    oled_show_string(0, 2, "HUB!");

    ret = nvs_flash_init();                                         

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    esp_flash_get_size(NULL, &flash_size);                         

    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "chipcores:%d\n",chip_info.cores);                     
    ESP_LOGI(TAG, "FLASH size:%ld MB flash\n",flash_size / (1024 * 1024)); 
    ESP_LOGI(TAG, "PSRAM size: %d bytes\n", esp_psram_get_size());        

    ESP_LOGI(TAG, "Initializing RA-01S...");

    gpio_config_t io_conf_lora_rst = {
        .pin_bit_mask = (1ULL << LORA_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_lora_rst);

    gpio_config_t io_conf_lora_dio0 = {
        .pin_bit_mask = (1ULL << LORA_DIO0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_lora_dio0);

    // SPI bus config
    spi_bus_config_t buscfg = {
        .mosi_io_num = LORA_MOSI,
        .miso_io_num = LORA_MISO,
        .sclk_io_num = LORA_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // SPI device config
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 2*1000*1000,  // 1 MHz for test
        .mode = 0,
        .spics_io_num = LORA_CS,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &lora_spi));

    // Reset RA-01S
    lora_reset();

    uint8_t status = sx126x_get_status();
    ESP_LOGI(TAG, "SX1262 Status = 0x%02X", status);

    if (status != 0x00 && status != 0xFF)
        ESP_LOGI(TAG, "RA-01SC-P is CONNECTED");
    else
        ESP_LOGE(TAG, "RA-01SC-P not detected");

    sx126x_set_standby();

    battery_adc_init();

    // Create mutex for OLED display synchronization
    oled_mutex = xSemaphoreCreateMutex();
    if (!oled_mutex) {
        ESP_LOGE(TAG, "Failed to create OLED mutex");
    }

    // Initialize SD card
    if (sd_card_init() == ESP_OK) {
        sd_list_files();
        // Example: write initial test file
        sd_write_file("test.txt", "ESP32-S3 SD Card Test\n");
    } else {
        ESP_LOGW(TAG, "SD card initialization failed");
    }

    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    uart_init_once(9600);

    ESP_LOGI(TAG, "UART ready on TX=%d RX=%d", UART_TX_PIN, UART_RX_PIN);
    xTaskCreate(gps_task, "gps_task", 8192, NULL, 10, NULL);

    while(1)
    {
        if (led_on_off) {
            /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 5, 5, 5));
            }
            /* Refresh the strip to send data */
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            // ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            // ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;

        float vbat = battery_get_voltage();
        g_battery_v = vbat;  // Update global for OLED display
        ESP_LOGI(TAG, "Battery = %.2f V", vbat);
        display_gps_battery_safe();  // Update OLED with current battery and last GPS data (rate-limited)
        vTaskDelay(1000);
    }
}

