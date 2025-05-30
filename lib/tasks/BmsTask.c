#include "bmsTask.h"
#include <driver/gpio.h>
#include <esp_log.h>

static StackType_t bmsTaskStack[4 * configMINIMAL_STACK_SIZE];
static Event bmsTaskQueue[10];
static Task bmsTask;

static EventTimer timer30s;

/**
 * BMS Register definitions
 */
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

/* ---------------------------------------------- */


// Forward declaration
static void EventLoop(uint8_t const event);

void BmsTask_Start()
{
    ESP_LOGI("BmsTask", "Starting BmsTask");

    TaskInit(&bmsTask, EventLoop);
    TaskStart(&bmsTask, 5 /*priority*/, bmsTaskQueue, sizeof(bmsTaskQueue), bmsTaskStack, sizeof(bmsTaskStack));

    // TimerInit(&timer1s, LED_TOGGLE, &bmsTask, true);
    // TimerStart(&timer1s, 1000);
}

static void EventLoop(uint8_t const event)
{
    ESP_LOGI("BmsTask", "Process Event %d", event);
    switch (event)
    {
    default:
        break;
    }
}