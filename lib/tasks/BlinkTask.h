#ifndef BLINK_TASK_H
#define BLINK_TASK_H

#include "TaskFramework.h"

typedef enum
{
    LED_TOGGLE,
    BLINKTASK_EVENT_N
} BlinkTaskEvent;

void BlinkTask_Start();

#endif /* BLINK_TASK_H */