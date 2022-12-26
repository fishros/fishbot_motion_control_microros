#include "fishbot_utils.h"

Rate::Rate(float rate)
{
    rate_ = rate;
    period_ = 1000 / rate;
}

void Rate::sleep()
{
    uint16_t last_run_time = millis() - last_loop_time_;
    if (last_run_time < period_)
    {
        delay(period_ - last_run_time);
    }
    last_loop_time_ = millis();
}