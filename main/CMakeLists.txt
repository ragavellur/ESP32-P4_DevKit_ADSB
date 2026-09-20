idf_component_register(
    SRCS
        "app_main.c"
        "rtl_pipeline.c"
        "adsb_state.c"
        "eth_netif.c"
        "http_radar.c"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "."
    EMBED_TXTFILES "radar_data/radar.html"
    REQUIRES
        adsb_decoder
        esp_rtl_sdr
        esp_eth
        esp_event
        esp_timer
        esp_http_server
        esp_netif
        nvs_flash
        log
)
