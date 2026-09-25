idf_component_register(
    SRCS "main.c" "ext_flash.c" "flash_log.c" "i2c_bus.c" "bq25792.c"
         "sensors.c" "solar.c" "board.c" "uplink.c" "isrg_cert.c"
         "uss.c" "wf280a.c" "gasflow.c" "gnss_bench.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer esp_wifi esp_netif esp_event mqtt nvs_flash
                  esp_http_client
)
