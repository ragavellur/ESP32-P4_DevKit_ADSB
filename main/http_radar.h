#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the embedded web server: / , /radar , /data/aircraft.json ,
 * /api/status. Binds INADDR_ANY (reachable on AP + Ethernet). */
esp_err_t http_radar_start(void);

#ifdef __cplusplus
}
#endif