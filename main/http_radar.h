#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the embedded web server: / , /radar , /data/aircraft.json ,
 * /api/status. Binds INADDR_ANY (reachable on AP + Ethernet). */
esp_err_t http_radar_start(void);

/* Receiver location (radar center). Persisted in NVS; exposed via
 * /api/location and embedded in /api/status and /data/aircraft.json. */
void http_radar_set_receiver(double lat, double lon);
void http_radar_get_receiver(double *lat, double *lon);

#ifdef __cplusplus
}
#endif