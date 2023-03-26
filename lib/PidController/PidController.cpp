#include "PidController.h"

PidController::PidController(float kp, float ki, float kd)
{
    update_pid(kp, ki, kd);
}

float PidController::update(float control)
{
    float error = target_ - control;
    derror_ = error_last_ - error;
    error_last_ = error;
    
    error_sum_ += error;
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_;
    if (error_sum_ < -1 * intergral_up_)
        error_sum_ = -1 * intergral_up_;

    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;
    // output limit
    if (output > out_max_)
        output = out_max_;
    if (output < out_mix_)
        output = out_mix_;


    return output;
}

void PidController::update_target(float target)
{
    target_ = target;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    last_output_ = 0.0f;
    target_ = 0.0f;
    out_mix_ = 0.0f;
    out_max_ = 0.0f;
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    error_sum_ = 0.0f;
    derror_ = 0.0f;
    error_last_ = 0.0f;
}

void PidController::out_limit(float out_mix, float out_max)
{
    out_mix_ = out_mix;
    out_max_ = out_max;
}
