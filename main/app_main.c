#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "eth_netif.h"
#include "rtl_pipeline.h"

static const char *TAG = "adsb_radar";

void app_main(void)
{
    ESP_LOGI(TAG, "ADS-B Radar boot (ESP32-P4)");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Internal RAM free:  %lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM free:         %lu",
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    ret = rtl_pipeline_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "rtl_pipeline_init failed: %s", esp_err_to_name(ret));
    }

    #ifdef CONFIG_ADSB_RADAR_ETH
    ret = eth_netif_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "eth_netif_start failed: %s", esp_err_to_name(ret));
    }
#else
    ESP_LOGW(TAG, "Ethernet netif disabled (CONFIG_ADSB_RADAR_ETH=n)");
#endif
}