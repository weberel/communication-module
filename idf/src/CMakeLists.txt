idf_component_register(
    SRCS "main.c" "ext_flash.c" "flash_log.c" "i2c_bus.c" "bq25792.c"
         "sensors.c" "solar.c" "board.c" "uplink.c" "isrg_cert.c"
         "uss.c" "wf280a.c" "gasflow.c" "gnss_bench.c" "devcfg.c" "ota.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer esp_wifi esp_netif esp_event mqtt nvs_flash
                  esp_http_client json
                  esp_https_ota app_update esp_partition mbedtls
)
# The image's app-descriptor version (from idf/version.txt) is also the FW_VERSION
# the code reports, so the two cannot drift apart. See config.h.
idf_build_get_property(project_ver PROJECT_VER)
target_compile_definitions(${COMPONENT_LIB} PRIVATE FW_VERSION="${project_ver}")
