#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "TaskFramework.h"

typedef enum
{
    SENSTASK_TIMER,
    SENSTASK_EVENT_N
} SensorTaskEvent;

void SensorTask_Start();

#endif /* SENSOR_TASK_H */