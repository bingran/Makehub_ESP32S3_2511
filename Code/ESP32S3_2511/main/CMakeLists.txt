idf_component_register(SRCS "ESP32S3_2511.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_common esp_wifi  nvs_flash led_strip 
                    PRIV_REQUIRES  spi_flash esp_adc esp_driver_gpio esp_psram ssd1306 driver fatfs)
