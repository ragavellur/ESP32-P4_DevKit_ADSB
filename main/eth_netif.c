#include <string.h>

#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "eth_netif.h"
#include "http_radar.h"

static const char *TAG = "eth_netif";

static bool s_eth_up;
static char s_ip[16];

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac[6];
    const char *link = NULL;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(*(esp_eth_handle_t *)event_data, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Ethernet link up (MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        link = "disconnected";
        break;
    case ETHERNET_EVENT_START:
        link = "started";
        break;
    case ETHERNET_EVENT_STOP:
        link = "stopped";
        break;
    default:
        break;
    }
    if (link) ESP_LOGI(TAG, "Ethernet %s", link);
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
    if (esp_netif_is_netif_up(e->esp_netif) == ESP_OK) {
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGW(TAG, "Ethernet IP: %s - open http://%s/", s_ip, s_ip);
    }
}

static void eth_worker(void *arg)
{
    esp_err_t e = esp_netif_init();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) goto fail;
    e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) goto fail;

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
    if (!eth_netif) {
        ESP_LOGE(TAG, "esp_netif_new failed");
        goto fail;
    }

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);
    if (!mac) {
        ESP_LOGE(TAG, "esp_eth_mac_new_esp32 failed");
        goto fail;
    }

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = -1;          /* auto-detect IP101GRI address */
    phy_cfg.reset_gpio_num = 51;          /* PHY_RST, active-low */
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);
    if (!phy) {
        ESP_LOGE(TAG, "esp_eth_phy_new_generic failed");
        goto fail;
    }

    esp_eth_handle_t eth = NULL;
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    e = esp_eth_driver_install(&eth_cfg, &eth);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install: %s", esp_err_to_name(e));
        goto fail;
    }

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth);
    if (!glue) {
        ESP_LOGE(TAG, "esp_eth_new_netif_glue failed");
        goto fail;
    }
    if (esp_netif_attach(eth_netif, glue) != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_attach failed");
        goto fail;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));

    esp_netif_dhcpc_start(eth_netif);
    ESP_LOGI(TAG, "starting Ethernet (DHCP client)");

    if (esp_eth_start(eth) != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start failed");
        goto fail;
    }

    s_eth_up = true;

    if (http_radar_start() != ESP_OK)
        ESP_LOGE(TAG, "http server failed");

    vTaskDelete(NULL);
    return;

fail:
    ESP_LOGE(TAG, "Ethernet bring-up failed");
    vTaskDelete(NULL);
}

esp_err_t eth_netif_start(void)
{
    if (s_eth_up) return ESP_OK;
    BaseType_t ok = xTaskCreate(eth_worker, "eth_netif", 8192, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}