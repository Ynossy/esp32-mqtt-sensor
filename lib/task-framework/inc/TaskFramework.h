#ifndef TASK_FRAMEWORK_H
#define TASK_FRAMEWORK_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

typedef void (*DispatchHandler)(uint8_t const e);

typedef uint16_t Event;

typedef struct
{
    TaskHandle_t thread;    /* private thread */
    StaticTask_t thread_cb; /* thread control-block (FreeRTOS static alloc) */

    QueueHandle_t queue;    /* private message queue */
    StaticQueue_t queue_cb; /* queue control-block (FreeRTOS static alloc) */

    DispatchHandler dispatch; /* pointer to the dispatch() function */
} Task;

void TaskInit(Task *const me, DispatchHandler dispatch);
void TaskStart(Task *const me,
               uint8_t prio, /* priority (1-based) */
               Event *queueSto,
               uint32_t queueLen,
               void *stackSto,
               uint32_t stackSize);
void TaskSendEvent(Task *const me, Event e);
void TaskSendEventISR(Task *const me, Event e,
                      BaseType_t *pxHigherPriorityTaskWoken);

/*---------------------------------------------------------------------------*/
/* Time Event facilities... */

/* Time Event class */
typedef struct
{
    Event event;            /* inherit Event */
    Task *task;             /* the AO that requested this EventTimer */
    TimerHandle_t timer;    /* private timer handle */
    StaticTimer_t timer_cb; /* timer control-block (FreeRTOS static alloc) */
    uint8_t reload;         /* enable reload for periodic timer */
} EventTimer;

void TimerInit(EventTimer *const me, Event sig, Task *act, uint8_t reload);
void TimerStart(EventTimer *const me, uint32_t millisec);
void TimerStop(EventTimer *const me);

#endif /* TASK_FRAMEWORK_H */
