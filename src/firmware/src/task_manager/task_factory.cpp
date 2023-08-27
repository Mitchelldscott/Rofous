#include "task_manager/task_factory.h"

Task* new_task(String key){
    if (key == LSM6DSOX_DRIVER_KEY) {
        return new LSM6DSOX();
    }
    else if (key == PWM_DRIVER_KEY) {
        return new PwmDriver();
    }
    else if (key == COMPFLTR_DRIVER_KEY) {
        return new ComplimentaryFilter();
    }
    else if (key == CONSTANT_DRIVER_KEY) {
        return new ConstTask();
    }
    else if (key == SINUSIOD_DRIVER_KEY) {
        return new SinTask();
    }
    else {
        return nullptr;
    }
}