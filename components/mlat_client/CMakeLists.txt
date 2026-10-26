idf_component_register(
    SRCS "src/mlat_client.c"
    INCLUDE_DIRS "include"
    REQUIRES adsb_decoder esp_rtl_sdr esp_timer nvs_flash esp_event esp_netif lwip log
)
