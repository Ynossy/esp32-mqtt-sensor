// wifi.h
#pragma once

/**
 * @file wifi.h
 * 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-04-15
 * 
 * @copyright Copyright (c) 2025
 * 
 * https://developer.espressif.com/blog/getting-started-with-wifi-on-esp-idf/
 */

#include "esp_err.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"

esp_err_t wifilib_init(void);

esp_err_t wifilib_connect(char* wifi_ssid, char* wifi_password);

esp_err_t wifilib_disconnect(void);

esp_err_t wifilib_deinit(void);
