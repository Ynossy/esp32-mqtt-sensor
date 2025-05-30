#include "BlinkTask.h"
#include <driver/gpio.h>
#include <esp_log.h>

#define BLINK_GPIO (gpio_num_t) GPIO_NUM_2

static StackType_t blinkTaskStack[4 * configMINIMAL_STACK_SIZE];
static Event blinkTaskQueue[10];
static Task blinkTask;

static EventTimer timer1s;

// Forward declaration
static void EventLoop(uint8_t const event);

void BlinkTask_Start()
{
    ESP_LOGI("BlinkTask", "Starting BlinkTask");

    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    TaskInit(&blinkTask, EventLoop);
    TaskStart(&blinkTask, 5 /*priority*/, blinkTaskQueue, sizeof(blinkTaskQueue), blinkTaskStack, sizeof(blinkTaskStack));

    TimerInit(&timer1s, LED_TOGGLE, &blinkTask, true);
    TimerStart(&timer1s, 1000);
}

static void EventLoop(uint8_t const event)
{
    // ESP_LOGI("BlinkTask", "Process Event %d", event);
    switch (event)
    {
    case LED_TOGGLE:
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 0);
        break;
    default:
        break;
    }
}