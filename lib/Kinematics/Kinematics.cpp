#include "Kinematics.h"

// 用于将欧拉角转换为四元数。
void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    // 传入机器人的欧拉角 roll、pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中    
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

// 用于将角度转换到 -π 到 π 的范围内
void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    // 如果 angle 大于 π，则将 out_angle 减去 2π
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    // 如果 angle 小于 -π，则将 out_angle 加上 2π
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

// 设置机器人的电机参数。该方法接受四个参数：电机的 ID、降速比、脉冲比和轮子直径。
void Kinematics::set_motor_param(uint8_t id, float reducation_ratio, uint16_t pulse_ration, float wheel_diameter)
{
    motor_param_[id].id = id;                                   // 机器人电机的 ID 
    motor_param_[id].reducation_ratio = reducation_ratio;       // 电机的减速比
    motor_param_[id].pulse_ration = pulse_ration;               // 电机的编码器脉冲比
    motor_param_[id].wheel_diameter = wheel_diameter;           // 电机的轮子直径
    // 计算机器人电机每个编码器脉冲对应的行驶距离
    // 公式为 轮子直径 乘以 圆周率 除以 减速比 乘以 编码器脉冲比
    motor_param_[id].per_pulse_distance = (wheel_diameter * PI) / (reducation_ratio * pulse_ration);    
    // 计算机器人电机的速度系数
    // 公式为轮子直径乘以圆周率除以减速比乘以编码器脉冲比乘以 1000000（即将单位转换为米每秒）
    motor_param_[id].speed_factor = (1000 * 1000) * (wheel_diameter * PI) / (reducation_ratio * pulse_ration);
    Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", id, motor_param_[id].per_pulse_distance, wheel_diameter, reducation_ratio, pulse_ration, motor_param_[id].speed_factor);
}

// 用于设置机器人的运动学参数。该方法接受一个参数：轮子之间的距离（即轮距），并将其保存到类中的 wheel_distance_ 变量中
void Kinematics::set_kinematic_param(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

// 用于更新机器人的电机编码器读数和计算电机速度。该方法接受三个参数：当前时间、电机1的编码器读数和电机2的编码器读数
void Kinematics::update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2)
{
    // 计算出自上次更新以来经过的时间 dt   
    uint32_t dt = current_time - motor_param_[0].last_update_time;
    // 电机1和电机2的编码器读数变化量 dtick1 和 dtick2。
    int32_t dtick1 = motor_tick1 - motor_param_[0].last_encoder_tick;
    int32_t dtick2 = motor_tick2 - motor_param_[1].last_encoder_tick;
    // 轮子速度计算
    motor_param_[0].motor_speed = dtick1 * ((double)motor_param_[0].speed_factor / dt);
    motor_param_[1].motor_speed = dtick2 * ((double)motor_param_[1].speed_factor / dt);
    // Serial.printf("motor_speed %d=%d*%d/%d\n", motor_param_[0].motor_speed, motor_param_[0].speed_factor, dtick1, dt);
    motor_param_[0].last_encoder_tick = motor_tick1;
    motor_param_[1].last_encoder_tick = motor_tick2;
    motor_param_[0].last_update_time = current_time;
    motor_param_[1].last_update_time = current_time;

    // 更新机器人里程计
    this->update_bot_odom_(dt);
}
// 机器人运动学模型的更新函数，用于更新机器人的位置和姿态信息
void Kinematics::update_bot_odom_(uint32_t dt)
{
    static float linear_speed, angular_speed;
    float dt_s = (float)(dt / 1000) / 1000;
    
    // 计算机器人的线速度和角速度。
    this->kinematic_forward(motor_param_[0].motor_speed, motor_param_[1].motor_speed, linear_speed, angular_speed);
    
    // 将计算出的角速度和线速度存储到 odom_ 结构体中。根据角速度和时间间隔，更新机器人的姿态信息 odom_.yaw
    odom_.angular_speed = angular_speed;
    odom_.linear_speed = linear_speed / 1000; // /1000（mm/s 转 m/s）

    odom_.yaw += odom_.angular_speed * dt_s;
    // 将角度值 odom_.yaw 转换到 -π 到 π 的范围内
    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);
    

    /*更新x和y轴上移动的距离*/
    // 计算机器人在 x 和 y 轴上移动的距离 delta_distance，单位为米
    float delta_distance = odom_.linear_speed * dt_s; // 单位m
    // 根据机器人的角度值 odom_.yaw 和移动距离 delta_distance，更新机器人的位置信息 odom_.x 和 odom_.y
    odom_.x += delta_distance * std::cos(odom_.yaw);
    odom_.y += delta_distance * std::sin(odom_.yaw);

    // Serial.printf("dist=%f,dt=%d,dts=%f,odom_.linear_speed=%f,odom_.yaw=%f,odom_.x=%f\n", delta_distance, dt, dt_s, odom_.linear_speed, odom_.yaw, odom_.x);
}

// 机器人运动学模型中的逆运动学函数，用于将机器人的线速度和角速度转换为两个轮子的转速。
void Kinematics::kinematic_inverse(float linear_speed, float angular_speed, float &out_wheel1_speed, float &out_wheel2_speed)
{
    // 计算左轮的转速 out_wheel1_speed，公式为 线速度 减去 角速度乘以轮距的一半
    out_wheel1_speed =
        linear_speed - (angular_speed * wheel_distance_) / 2.0;
    // 计算右轮的转速 out_wheel2_speed，公式为 线速度 加上 角速度乘以轮距的一半。
    out_wheel2_speed =
        linear_speed + (angular_speed * wheel_distance_) / 2.0;
}

// 用于计算机器人的线速度和角速度。
void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float &linear_speed, float &angular_speed)
{
    // 两轮转速之和除以2
    linear_speed = (wheel1_speed + wheel2_speed) / 2.0;
    // 两轮转速之差除以轮距
    angular_speed =
        (wheel2_speed - wheel1_speed) / wheel_distance_;
}

// 用于获取机器人的位置和姿态信息
odom_t &Kinematics::odom()
{
    // 调用 Euler2Quaternion 函数，将机器人的欧拉角 yaw 转换为四元数 quaternion。
    Kinematics::Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    // 返回机器人的位置和姿态信息 odom_
    return odom_;
}

// 传入电机的编号 id。返回该编号电机的速度
float Kinematics::motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}