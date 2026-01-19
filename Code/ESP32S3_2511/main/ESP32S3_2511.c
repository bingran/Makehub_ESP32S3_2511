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

/* I2C configuration */
#define I2C_MASTER_PORT        I2C_NUM_0
#define I2C_MASTER_SDA_IO      10
#define I2C_MASTER_SCL_IO      9
#define I2C_MASTER_FREQ_HZ     100000
/* OLED */
#define OLED_ADDR      0x3D
#define OLED_WIDTH     128
#define OLED_HEIGHT    64

/* SSD1306 commands */
#define OLED_CMD       0x00
#define OLED_DATA      0x40

static uint8_t framebuffer[OLED_WIDTH * OLED_HEIGHT / 8];


static const char *TAG = "MAIN";

static bool uart_ready = false;

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

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0));
}

static void oled_write(uint8_t control, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void oled_cmd(uint8_t cmd)
{
    oled_write(OLED_CMD, &cmd, 1);
}

static void oled_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    oled_cmd(0xAE); // Display OFF
    oled_cmd(0xD5); // Set display clock divide
    oled_cmd(0x80);

    oled_cmd(0xA8); // Set multiplex
    oled_cmd(0x1F); // <<< 0x1F = 32 rows (CRITICAL)

    oled_cmd(0xD3); // Display offset
    oled_cmd(0x00);

    oled_cmd(0x40); // Start line

    oled_cmd(0x8D); // Charge pump
    oled_cmd(0x14);

    oled_cmd(0x20); // Memory mode
    oled_cmd(0x00); // Horizontal

    oled_cmd(0xA1); // Segment remap
    oled_cmd(0xC8); // COM scan direction

    oled_cmd(0xDA); // COM pins
    oled_cmd(0x02); // <<< DIFFERENT FROM SSD1306 64

    oled_cmd(0x81); // Contrast
    oled_cmd(0xCF); // <<< HIGH contrast (important)

    oled_cmd(0xD9); // Precharge
    oled_cmd(0xF1);

    oled_cmd(0xDB); // VCOM detect
    oled_cmd(0x40);

    oled_cmd(0xA4); // Resume RAM
    oled_cmd(0xA6); // Normal display

    oled_cmd(0xAF); // Display ON
}

static void oled_clear(void)
{
    memset(framebuffer, 0x00, sizeof(framebuffer));
}

/* ---------------- FONT (5x7) ---------------- */

static const uint8_t font5x7[][5] = {
    {0x7E,0x11,0x11,0x11,0x7E}, // H
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x08,0x08,0x08,0x7F}, // L
    {0x7F,0x49,0x49,0x49,0x41}, // O
    {0x00,0x00,0x00,0x00,0x00}, // space
};

static void oled_draw_char(int x, int page, const uint8_t *c)
{
    memcpy(&framebuffer[page * OLED_WIDTH + x], c, 5);
}

static void oled_draw_hello(void)
{
    oled_draw_char(10, 3, font5x7[0]); // H
    oled_draw_char(16, 3, font5x7[1]); // E
    oled_draw_char(22, 3, font5x7[2]); // L
    oled_draw_char(28, 3, font5x7[2]); // L
    oled_draw_char(34, 3, font5x7[3]); // O
}

/* ---------------- Display ---------------- */

static void oled_update(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        oled_write(OLED_DATA,
                   &framebuffer[page * OLED_WIDTH],
                   OLED_WIDTH);
    }
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

// Task to continuously read GPS data
static void gps_task(void *arg)
{
    int baud = detect_gps_baud();
    if (baud < 0)
    {
        ESP_LOGE(TAG, "GPS not detected");
        vTaskDelete(NULL);
        return;
    }

    uint8_t data[BUF_SIZE];
    ESP_LOGI(TAG, "Starting GPS read at %d baud", baud);

    while (1)
    {
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
        if (len > 0)
        {
            data[len] = 0;
            ESP_LOGI(TAG, "GPS: %s", data);
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

    led_strip_handle_t led_strip = configure_led();
    bool led_on_off = false;

    // init_oled();
    
    uart_init_once(9600);

    i2c_master_init();
    uint8_t data[128];
    memset(data, 0xFF, sizeof(data));

    for (uint8_t page = 0; page < 4; page++) {   // 4 pages = 32px
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        oled_write(0x40, data, 128);
}   
    ESP_LOGI(TAG, "Display updated");


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
            ESP_LOGI(TAG, "LED ON!");
        } else {
            /* Set all LED off to clear all pixels */
            ESP_ERROR_CHECK(led_strip_clear(led_strip));
            ESP_LOGI(TAG, "LED OFF!");
        }

        led_on_off = !led_on_off;

        float vbat = battery_get_voltage();
        ESP_LOGI(TAG, "Battery = %.2f V", vbat);
        vTaskDelay(1000);
    }
}

