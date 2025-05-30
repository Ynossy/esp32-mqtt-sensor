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

#include "smbus.h"

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

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       50000
#define I2C_MASTER_SDA_IO        GPIO_NUM_21
#define I2C_MASTER_SCL_IO        GPIO_NUM_22

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}


esp_err_t i2c_read_bytes(const smbus_info_t * smbus_info, uint8_t command, uint8_t * data, size_t len)
{
    // Protocol: [S | ADDR | Wr | As | COMMAND | As | Sr | ADDR | Rd | As | (DATAs | A){*len-1} | DATAs | N | P]
    esp_err_t err = ESP_FAIL;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, smbus_info->address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, smbus_info->address << 1 | I2C_MASTER_READ, true);
    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, 0);
    }
    i2c_master_read_byte(cmd, &data[len - 1], 1);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(smbus_info->i2c_port, cmd, smbus_info->timeout);
    i2c_cmd_link_delete(cmd);

    return err;
}

void smbus_task(void *pvParameter)
{
     // Set up I2C
     i2c_master_init();
     i2c_port_t i2c_num = I2C_MASTER_NUM;
     uint8_t address = 0x55;
 
     // Set up the SMBus
     smbus_info_t * smbus_info = smbus_malloc();
     smbus_init(smbus_info, i2c_num, address);
     
     while (1)
     {
        for(uint8_t query = 0; query<0x77; query++) // 7 bit address
        {
            // uint16_t reg = 0;
            // esp_err_t r = smbus_read_word(smbus_info, query, &reg);
            uint8_t reg = 0;
            esp_err_t r = i2c_read_bytes(smbus_info, query, &reg, 1);
            if(r == ESP_OK)
            {
                ESP_LOGI("SMBUS", "0x%02x: %d",(uint16_t)query, (uint16_t)reg);
            }
            else
            {
                ESP_LOGI("SMBUS", "Failed to read smbus for address %d: %d", (uint16_t)address ,r);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
     }
}


// static esp_err_t i2c_master_read_slave_reg(uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
// {
//     if (size == 0) {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     // first, send device address (indicating write) & register to be read
//     i2c_master_write_byte(cmd, ( i2c_addr << 1 ), true);
//     // send register we want
//     i2c_master_write_byte(cmd, i2c_reg, true);
//     // Send repeated start
//     i2c_master_start(cmd);
//     // now send device address (indicating read) & read data
//     i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | I2C_MASTER_READ, true);
//     if (size > 1) {
//         i2c_master_read(cmd, data_rd, size - 1, false);
//     }
//     i2c_master_read_byte(cmd, data_rd + size - 1, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

// void battery_task(void *pvParameter)
// {
//     i2c_master_init();
//     vTaskDelay(pdMS_TO_TICKS(500));
//     while(1)
//     {
//         ESP_LOGI(TAG, "Query I2C device:");
//         uint8_t data[2] = {0, 0};
//         esp_err_t ret = i2c_master_read_slave_reg(0x55, 0x09, data, 2);
//         ESP_LOGI(TAG, "Read value: %d, %d", (uint16_t)data[1], (uint16_t)data[0]);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }


// void i2cscan_task(void *pvParameter)
// {
//     i2c_master_init();
//     vTaskDelay(pdMS_TO_TICKS(500));
    
//     for (uint8_t address = 0x03; address < 0x78; address++) {
//         ESP_LOGI(TAG, "Scanning I2C bus: %d", (uint16_t)address);
//         i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//         i2c_master_start(cmd);
//         i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
//         i2c_master_stop(cmd);
//         esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
//         i2c_cmd_link_delete(cmd);
        
//         vTaskDelay(pdMS_TO_TICKS(500));
//         if (ret == ESP_OK) {
//             ESP_LOGI(TAG, "Found device at 0x%02X\n", address);
//         }
//     }
    
//     ESP_LOGI(TAG, "Scan ended!");
//     while(true);
// }

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

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO (gpio_num_t) GPIO_NUM_2

void blink_task(void *pvParameter)
{
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Blink\n");
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

    // ESP_ERROR_CHECK(nvs_flash_init());
    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    mqtt_app_start(BROKER_URL);
}

void app_main()
{
    ESP_LOGI(TAG, "Starting Application...");
    // xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    // xTaskCreate(&ds18x20_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
    // xTaskCreate(&smbus_task, TAG, configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    BlinkTask_Start();

    // start_wifi();
    // start_mqtt();
}