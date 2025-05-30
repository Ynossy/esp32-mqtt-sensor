#include "TaskFramework.h"
#include <esp_log.h>

/*..........................................................................*/
void TaskInit(Task *const me, DispatchHandler dispatch)
{
    me->dispatch = dispatch; /* assign the dispatch handler */
}

/*..........................................................................*/
/* thread function for all Tasks (FreeRTOS task signature) */
static void TaskLoop(void *pvParameters)
{
    Task *me = (Task *)pvParameters;

    configASSERT(me); /* Task object must be provided */
    for (;;)
    {            /* for-ever "superloop" */
        Event e; /* pointer to event object ("message") */

        /* wait for any event and receive it into object 'e' */
        xQueueReceive(me->queue, &e, portMAX_DELAY); /* BLOCKING! */

        /* dispatch event to the Task object 'me' */
        (*me->dispatch)(e); /* NO BLOCKING! */
    }
}

/*..........................................................................*/
void TaskStart(Task *const me,
               uint8_t prio, /* priority (1-based) */
               Event *queueSto,
               uint32_t queueLen,
               void *stackSto,
               uint32_t stackSize)
{
    StackType_t *stk_sto = stackSto;
    uint32_t stk_depth = (stackSize / sizeof(StackType_t));

    me->queue = xQueueCreateStatic(
        queueLen,            /* queue length - provided by user */
        sizeof(Event),       /* item size */
        (uint8_t *)queueSto, /* queue storage - provided by user */
        &me->queue_cb);      /* queue control block */
    if (!me->queue)
    {
        ESP_LOGE("TaskFramework", "Failed to create queue!");
        return;
    }

    me->thread = xTaskCreateStatic(
        &TaskLoop,               /* the thread function */
        "AO",                    /* the name of the task */
        stk_depth,               /* stack depth */
        me,                      /* the 'pvParameters' parameter */
        prio + tskIDLE_PRIORITY, /* FreeRTOS priority */
        stk_sto,                 /* stack storage - provided by user */
        &me->thread_cb);         /* task control block */
    if (!me->queue)
    {
        ESP_LOGE("TaskFramework", "Failed to create task!");
        return;
    }
}

/*..........................................................................*/
void TaskSendEvent(Task *const me, Event e)
{
    BaseType_t status = xQueueSendToBack(me->queue, (void *)&e, (TickType_t)0);
    configASSERT(status == pdTRUE);
    if (status == pdFALSE)
    {
        ESP_LOGE("TaskFramework", "Failed to send event!");
    }
}

/*..........................................................................*/
void TaskSendEventISR(Task *const me, Event e,
                      BaseType_t *pxHigherPriorityTaskWoken)
{
    BaseType_t status = xQueueSendToBackFromISR(me->queue, (void *)&e,
                                                pxHigherPriorityTaskWoken);
    if (status == pdFALSE)
    {
        ESP_LOGE("TaskFramework", "Failed to send event from ISR!");
    }
}

/*--------------------------------------------------------------------------*/
/* Time Event services... */
static void EventTimerCallback(TimerHandle_t xTimer);

/*..........................................................................*/
void TimerInit(EventTimer *const me, Event event, Task *task, uint8_t reload)
{
    /* no critical section because it is presumed that all EventTimers
     * are created *before* multitasking has started.
     */
    me->event = event;
    me->task = task;

    /* Create a timer object */
    me->timer = xTimerCreateStatic("TE", 1U, reload, me,
                                   EventTimerCallback, &me->timer_cb);
    if (!me->timer)
    {
        ESP_LOGE("TaskFramework", "Failed to init timer!");
    }
}

/*..........................................................................*/
void TimerStart(EventTimer *const me, uint32_t millisec)
{
    TickType_t ticks;
    BaseType_t status;
    BaseType_t xHigherPriorityTaskWoken;

    ticks = (millisec / portTICK_PERIOD_MS);
    if (ticks == 0U)
    {
        ticks = 1U;
    }

    if (xPortInIsrContext() == pdTRUE)
    {
        xHigherPriorityTaskWoken = pdFALSE;

        status = xTimerChangePeriodFromISR(me->timer, ticks,
                                           &xHigherPriorityTaskWoken);
        if (status != pdPASS)
        {
            ESP_LOGE("TaskFramework", "Failed to start timer!");
        }

        // not supported on esp32
        // portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        status = xTimerChangePeriod(me->timer, ticks, 0);
        if (status != pdPASS)
        {
            ESP_LOGE("TaskFramework", "Failed to start timer!");
        }
    }
}

/*..........................................................................*/
void TimerStop(EventTimer *const me)
{
    BaseType_t xHigherPriorityTaskWoken;
    BaseType_t status;

    if (xPortInIsrContext() == pdTRUE)
    {
        xHigherPriorityTaskWoken = pdFALSE;
        status = xTimerStopFromISR(me->timer, &xHigherPriorityTaskWoken);
        if (status != pdPASS)
        {
            ESP_LOGE("TaskFramework", "Failed to stop timer!");
        }

        // not supported on esp32
        // portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        status = xTimerStop(me->timer, 0);
        if (status != pdPASS)
        {
            ESP_LOGE("TaskFramework", "Failed to stop timer!");
        }
    }
}

/*..........................................................................*/
/* Use this macro to get the container of EventTimer struct
 *  since xTimer pointing to timer_cb
 */
#define GET_TIME_EVENT_HEAD(ptr) \
    (EventTimer *)((uintptr_t)(ptr) - offsetof(EventTimer, timer_cb))

static void EventTimerCallback(TimerHandle_t xTimer)
{
    /* Also can use pvTimerGetTimerID(xTimer) */
    EventTimer *const t = GET_TIME_EVENT_HEAD(xTimer);

    /* Callback always called from non-interrupt context so no need
     * to check xPortIsInsideInterrupt
     */
    ESP_LOGI("TaskFramework", "Send timer event.");
    TaskSendEvent(t->task, t->event);
}
