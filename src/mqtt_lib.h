#pragma once

#include "esp_event.h"
#include <stdint.h>

/**
 *  https://github.com/espressif/esp-idf/tree/master/examples/protocols/mqtt/tcp
 * 
 */

void log_error_if_nonzero(const char *message, int error_code);
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_app_start(char* broker_url);

void mqtt_send(char* topic, char* payload);
