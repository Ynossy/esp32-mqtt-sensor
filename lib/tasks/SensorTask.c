#include "sensorTask.h"
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

static StackType_t sensorTaskStack[4 * configMINIMAL_STACK_SIZE];
static Event sensorTaskQueue[10];
static Task sensorTask;

static EventTimer timerBms;
static EventTimer timerAdc;

/**
 * BMS Register definitions
 */
const uint8_t MAX_REG_LENGTH = 2;

const uint8_t REG_TEMPERATURE = 0x06;
const uint8_t REG_TEMPERATURE_LEN = 2;

const uint8_t REG_VOLTAGE = 0x08;
const uint8_t REG_VOLTAGE_LEN = 2;

const uint8_t REG_CURRENT = 0x14;
const uint8_t REG_CURRENT_LEN = 2;

const uint8_t REG_SOC_MAH = 0x20;
const uint8_t REG_SOC_MAH_LEN = 2;

const uint8_t REG_SOC_PCT = 0x2c;
const uint8_t REG_SOC_PCT_LEN = 1;

const uint8_t REG_SOC_MAX = 0x18;
const uint8_t REG_SOC_MAX_LEN = 2;

/* -------------------- Periphery -------------------------- */
// I2C
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x55,
    .scl_speed_hz = 50000,
};

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

// ADC
#define ADC_V_GPIO (gpio_num_t) GPIO_NUM_25

// ADC_CHANNEL_0 -> A0/IO36
// ADC_CHANNEL_3 -> A1/IO39
// ADC_CHANNEL_6 -> A2/IO34
// ADC_CHANNEL_7 -> A3/IO37

static esp_adc_cal_characteristics_t adc_chars;
static const adc_atten_t adc_atten = ADC_ATTEN_DB_12;
static const adc_unit_t adc_unit = ADC_UNIT_1;

/* ---------------------------------------------- */

// Forward declaration
static void EventLoop(uint8_t const event);
static esp_err_t ReadRegister(uint8_t address, uint8_t *data, size_t len);
static void ReadBmsRegisters();
static void ReadAdc1();

// local variables
static uint16_t bms_value_temperature = 0;         // millikelvin
static uint16_t bms_value_voltage = 0;             // millivolt
static int16_t bms_value_current = 0;              // milliamps
static uint16_t bms_value_state_of_charge = 0;     // mAh
static uint16_t bms_value_max_state_of_charge = 0; // mAh
static uint16_t bms_value_soc_percent = 0;         // 0-100%

void SensorTask_Start()
{
    ESP_LOGI("SensorTask", "Starting SensorTask");

    // I2C
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    // voltage supply for measurement
    gpio_set_direction(ADC_V_GPIO, GPIO_MODE_OUTPUT);

    // ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL_0, adc_atten);
    adc1_config_channel_atten(ADC_CHANNEL_3, adc_atten);
    adc1_config_channel_atten(ADC_CHANNEL_6, adc_atten);
    adc1_config_channel_atten(ADC_CHANNEL_7, adc_atten);
    // Characterize ADC
    esp_adc_cal_characterize(adc_unit, adc_atten, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    TaskInit(&sensorTask, EventLoop);
    TaskStart(&sensorTask, 5 /*priority*/, sensorTaskQueue, sizeof(sensorTaskQueue) / sizeof(sensorTaskQueue[0]), sensorTaskStack, sizeof(sensorTaskStack));

    TimerInit(&timerBms, SENSTASK_TIMER_BMS, &sensorTask, true);
    TimerStart(&timerBms, 10000);

    TimerInit(&timerAdc, SENSTASK_TIMER_ADC, &sensorTask, true);
    TimerStart(&timerAdc, 1000);
}

static void EventLoop(uint8_t const event)
{
    ESP_LOGI("SensorTask", "Process Event %d", event);
    switch (event)
    {
    case SENSTASK_TIMER_BMS:
    {
        ReadBmsRegisters();
    }
    break;
    case SENSTASK_TIMER_ADC:
    {
        gpio_set_level(ADC_V_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        ReadAdc1();
        gpio_set_level(ADC_V_GPIO, 0);
    }
    break;
    default:
        break;
    }
}

static esp_err_t ReadRegister(uint8_t address, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &address, 1, data, len, -100);
}

static void ReadBmsRegisters()
{
    uint8_t buffer[MAX_REG_LENGTH]; // all values are either 1 or 2 bytes long

    // bms_value_temperature
    esp_err_t ret = ReadRegister(REG_TEMPERATURE, buffer, REG_TEMPERATURE_LEN);
    if (ret == ESP_OK)
    {
        bms_value_temperature = *(uint16_t *)buffer;
        ESP_LOGI("SensorTask", "Temperature: %u", bms_value_temperature);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read temperature, code: %d", ret);
        return;
    }

    // bms_value_voltage
    ret = ReadRegister(REG_VOLTAGE, buffer, REG_VOLTAGE_LEN);
    if (ret == ESP_OK)
    {
        bms_value_voltage = *(uint16_t *)buffer;
        ESP_LOGI("SensorTask", "Voltage: %u", bms_value_voltage);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read voltage, code: %d", ret);
        return;
    }

    // bms_value_current
    ret = ReadRegister(REG_CURRENT, buffer, REG_CURRENT_LEN);
    if (ret == ESP_OK)
    {
        bms_value_current = *(int16_t *)buffer;
        ESP_LOGI("SensorTask", "Current: %u", bms_value_current);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read current, code: %d", ret);
        return;
    }

    // bms_value_state_of_charge
    ret = ReadRegister(REG_SOC_MAH, buffer, REG_SOC_MAH_LEN);
    if (ret == ESP_OK)
    {
        bms_value_state_of_charge = *(uint16_t *)buffer;
        ESP_LOGI("SensorTask", "SOC mAh: %u", bms_value_state_of_charge);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read soc mAh, code: %d", ret);
        return;
    }

    // bms_value_max_state_of_charge
    ret = ReadRegister(REG_SOC_MAX, buffer, REG_SOC_MAX_LEN);
    if (ret == ESP_OK)
    {
        bms_value_max_state_of_charge = *(uint16_t *)buffer;
        ESP_LOGI("SensorTask", "Max soc: %u", bms_value_max_state_of_charge);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read max soc, code: %d", ret);
        return;
    }

    // bms_value_soc_percent
    ret = ReadRegister(REG_SOC_PCT, buffer, REG_SOC_PCT_LEN);
    if (ret == ESP_OK)
    {
        bms_value_soc_percent = buffer[0];
        ESP_LOGI("SensorTask", "Soc pct: %u", bms_value_soc_percent);
    }
    else
    {
        ESP_LOGE("SensorTask", "Failed to read soc pct, code: %d", ret);
        return;
    }
}

static void ReadAdc1()
{
    int adc_raw_0 = adc1_get_raw(ADC_CHANNEL_0);
    int adc_raw_1 = adc1_get_raw(ADC_CHANNEL_3);
    int adc_raw_2 = adc1_get_raw(ADC_CHANNEL_6);
    int adc_raw_3 = adc1_get_raw(ADC_CHANNEL_7);
    ESP_LOGI("SensorTask", "raw adc values : %d %d %d %d", adc_raw_0, adc_raw_1, adc_raw_2, adc_raw_3);
    uint32_t voltage_0 = esp_adc_cal_raw_to_voltage(adc_raw_0, &adc_chars);
    uint32_t voltage_1 = esp_adc_cal_raw_to_voltage(adc_raw_1, &adc_chars);
    uint32_t voltage_2 = esp_adc_cal_raw_to_voltage(adc_raw_2, &adc_chars);
    uint32_t voltage_3 = esp_adc_cal_raw_to_voltage(adc_raw_3, &adc_chars);
    ESP_LOGI("SensorTask", "cali data: %lu %lu %lu %lu mV", voltage_0, voltage_1, voltage_2, voltage_3);
}