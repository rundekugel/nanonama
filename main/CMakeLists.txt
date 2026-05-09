idf_component_register(
    SRCS "main.c" "ssd1306.c" "mqtt_simple.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "index.html" "wifi.html" "config.html"
    REQUIRES
        esp_wifi
        esp_event
        nvs_flash
        esp_http_server
        tcp_transport
        driver
        esp_driver_gpio
        esp_timer
        esp_netif
        mbedtls
)
