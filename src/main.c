/* Blink Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "ds18x20.h"
#include <esp_log.h>
#include <esp_err.h>
#include "esp_wifi.h"
#include "wifilib.h"
#include "mqtt_lib.h"
#include "secrets.h"
#include "BlinkTask.h"
#include "SensorTask.h"

// #include "smbus.h"

#ifndef WIFI_SSID
#error "Please set WIFI_SSID in secrets.h"
#endif
#ifndef WIFI_PASSWORD
#error "Please set WIFI_PASSWORD in secrets.h"
#endif


#define BROKER_URL "mqtt://192.168.0.213:1883"

static const gpio_num_t SENSOR_GPIO = GPIO_NUM_25; // IO25/D2
#define DS18B20_OFFSET -1.0

static const char *TAG = "ds18x20_test";

void ds18x20_task(void *pvParameter)
{
    // Make sure that the internal pull-up resistor is enabled on the GPIO pin
    // so that one can connect up a sensor without needing an external pull-up.
    // (Note: The internal (~47k) pull-ups of the ESP do appear to work, at
    // least for simple setups (one or two sensors connected with short leads),
    // but do not technically meet the pull-up requirements from the ds18x20
    // datasheet and may not always be reliable. For a real application, a proper
    // 4.7k external pull-up resistor is recommended instead!)
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);
    static const int MAX_SENSORS = 8;
    ds18x20_addr_t addrs[MAX_SENSORS];
    size_t sensor_count = 0;

    float temperature;
    esp_err_t res;
    res = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS, &sensor_count);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Sensors scan error %d (%s)", res, esp_err_to_name(res));
    }
    if (!sensor_count)
    {
        ESP_LOGW(TAG, "No sensors detected!");
    }
    ESP_LOGI(TAG, "%d sensors detected", sensor_count);

    float average_temperature = 0.0f;
    uint32_t send_counter = 0;
    while (1)
    {
        res = ds18x20_measure_and_read(SENSOR_GPIO, addrs[0], &temperature);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not read from sensor %08" PRIx32 "%08" PRIx32 ": %d (%s)",
                     (uint32_t)(addrs[0] >> 32), (uint32_t)addrs[0], res, esp_err_to_name(res));
        }
        else
        {

            ESP_LOGI(TAG, "Sensor %08" PRIx32 "%08" PRIx32 ": %.2f°C",
                     (uint32_t)(addrs[0] >> 32), (uint32_t)addrs[0], temperature);
        }

        average_temperature += temperature + DS18B20_OFFSET;
        if(++send_counter%30 == 0) {
            char payload_json[32];
            snprintf(payload_json, sizeof(payload_json), "{\"temperature\": %.2f}", average_temperature/30.0f);

            mqtt_send("/temperature", payload_json);
            average_temperature = 0.0;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void start_wifi()
{
    ESP_LOGI(TAG, "Starting tutorial...");
    ESP_ERROR_CHECK(wifilib_init());

    esp_err_t ret = wifilib_connect(WIFI_SSID, WIFI_PASSWORD);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network");
    }

    wifi_ap_record_t ap_info;
    ret = esp_wifi_sta_get_ap_info(&ap_info);
    if (ret == ESP_ERR_WIFI_CONN)
    {
        ESP_LOGE(TAG, "Wi-Fi station interface not initialized");
    }
    else if (ret == ESP_ERR_WIFI_NOT_CONNECT)
    {
        ESP_LOGE(TAG, "Wi-Fi station is not connected");
    }
    else
    {
        ESP_LOGI(TAG, "--- Access Point Information ---");
        ESP_LOG_BUFFER_HEX("MAC Address", ap_info.bssid, sizeof(ap_info.bssid));
        ESP_LOG_BUFFER_CHAR("SSID", ap_info.ssid, sizeof(ap_info.ssid));
        ESP_LOGI(TAG, "Primary Channel: %d", ap_info.primary);
        ESP_LOGI(TAG, "RSSI: %d", ap_info.rssi);

        ESP_LOGI(TAG, "Disconnecting in 5 seconds...");
        // vTaskDelay(pdMS_TO_TICKS(5000));
    }

    // ESP_ERROR_CHECK(wifilib_disconnect());

    // ESP_ERROR_CHECK(wifilib_deinit());

    ESP_LOGI(TAG, "End of tutorial...");
}

void start_mqtt()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    // already handled by wifi init
    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    mqtt_app_start(BROKER_URL);
}

void app_main()
{
    ESP_LOGI(TAG, "Starting Application...");
    // xTaskCreate(&ds18x20_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    BlinkTask_Start();
    SensorTask_Start();

    // start_wifi();
    // start_mqtt();
}