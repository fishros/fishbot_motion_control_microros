#include "PidController.h"

// PID 控制器的构造函数，用于初始化 PID 控制器的比例系数、积分系数和微分系数
PidController::PidController(float kp, float ki, float kd)
{
    update_pid(kp, ki, kd);
}

float PidController::update(float control)
{
    // 传入反馈值 control，计算出误差 error = 目标值 - 反馈值
    float error = target_ - control;
    // 计算出本次误差与上一次误差的差值 derror
    derror_ = error_last_ - error;
    // 更新上一次误差值为当前误差值
    error_last_ = error;

    // 计算出误差积分 error_sum，用于处理积分项
    error_sum_ += error;
    // 如果误差积分超过积分上限 intergral_up_，将其限制在积分上限范围内
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_;
    if (error_sum_ < -1 * intergral_up_)
        error_sum_ = -1 * intergral_up_;

    /* kp_、ki_、kd_ 分别为比例系数、积分系数和微分系数，控制 PID 控制器的响应特性 */
    // 根据比例系数 kp_、积分系数 ki_ 和微分系数 kd_，计算出 PID 控制器的输出量 output
    /*  比例项 kp_ * error：表示输出量与误差成正比，用于快速响应目标值的变化。
        积分项 ki_ * error_sum_：表示输出量与误差的积分成正比，用于消除系统的静态误差。
        微分项 kd_ * derror_：表示输出量与误差的微分成正比，用于消除系统的动态误差。
    */
    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;

    // 如果输出量超过输出上限 out_max_，将其限制在输出上限范围内。
    if (output > out_max_)
        output = out_max_;
    // 如果输出量低于输出下限 out_mix_，将其限制在输出下限范围内。
    if (output < out_mix_)
        output = out_mix_;

    return output;
}
// 更新 PID 控制器的目标值
void PidController::update_target(float target)
{
    target_ = target;
}
// 用于更新 PID 控制器的比例系数、积分系数和微分系数的函数。
void PidController::update_pid(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
// 重置 PID 控制器的状态的函数
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
// 设置 PID 控制器的输出限幅范围的函数
void PidController::out_limit(float out_mix, float out_max)
{
    out_mix_ = out_mix;
    out_max_ = out_max;
}
