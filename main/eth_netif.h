#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the onboard 100M Ethernet (ESP32-P4 internal EMAC + IP101GRI RMII
 * PHY, Waveshare ESP32-P4-ETH) and start the radar web server on the wired
 * network. Non-blocking: spawns a worker task so the RTL-SDR pipeline start is
 * never held up. */
esp_err_t eth_netif_start(void);

#ifdef __cplusplus
}
#endif