idf_component_register(SRCS "ESP32S3_2511.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_common esp_wifi  nvs_flash led_strip
                    PRIV_REQUIRES esp_psram spi_flash)
