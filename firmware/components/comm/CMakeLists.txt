idf_component_register(
    SRCS "line_reader.c" "transport_uart.c" "transport_bt.c" "transport_http.c"
    INCLUDE_DIRS "."
    EMBED_TXTFILES "web_page.html" "leg_model.js"
    REQUIRES bsp control bt nvs_flash esp_wifi esp_netif esp_event esp_http_server
)
