#include "Kinematics.h"

void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

void Kinematics::set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter)
{
    motor_param_[id].id = id;
    motor_param_[id].reducation_ratio = reducation_ratio;
    motor_param_[id].pulse_ration = pulse_ration;
    motor_param_[id].wheel_diameter = wheel_diameter;
    motor_param_[id].per_pulse_distance = (wheel_diameter * PI) / (reducation_ratio * pulse_ration);
    motor_param_[id].speed_factor = (1000 * 1000) * (wheel_diameter * PI) / (reducation_ratio * pulse_ration);
    // 给初值
    motor_param_[id].last_encoder_tick = 0;

    // Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", id, motor_param_[id].per_pulse_distance, wheel_diameter, reducation_ratio, pulse_ration, motor_param_[id].speed_factor);
}

void Kinematics::set_kinematic_param(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}

void Kinematics::update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2, int32_t motor_tick3, int32_t motor_tick4)
{
    uint32_t dt = current_time - motor_param_[0].last_update_time;
    static int32_t dticks[4];
    static int32_t motor_ticks[4];
    static int8_t index;

    motor_ticks[0] = motor_tick1;
    motor_ticks[1] = motor_tick2;
    motor_ticks[2] = motor_tick3;
    motor_ticks[3] = motor_tick4;

    for (int index = 0; index < 4; index++)
    {
        // ticks数量计算
        dticks[index] = motor_ticks[index] - motor_param_[index].last_encoder_tick;
        // 轮子速度计算
        motor_param_[index].motor_speed = dticks[index] * (motor_param_[index].speed_factor / dt);
        motor_param_[index].last_encoder_tick = motor_ticks[index];
        motor_param_[index].last_update_time = current_time;
    }

    // 更新机器人里程计
    this->update_bot_odom_(dt);
}

void Kinematics::update_bot_odom_(uint32_t dt)
{
    static float linear_x_speed, linear_y_speed, angular_speed;
    float dt_s = (float)(dt / 1000) / 1000;

    this->kinematic_forward(motor_param_[0].motor_speed,
                            motor_param_[1].motor_speed,
                            motor_param_[2].motor_speed,
                            motor_param_[3].motor_speed,
                            linear_x_speed,
                            linear_y_speed,
                            angular_speed);

    odom_.angular_speed = angular_speed;
    odom_.linear_x_speed = linear_x_speed / 1000; // /1000（mm/s 转 m/s）
    odom_.linear_y_speed = linear_y_speed / 1000; // /1000（mm/s 转 m/s）

    /*更新x和y轴上移动的距离*/
    odom_.x += odom_.linear_x_speed * cos(odom_.yaw) * dt_s + odom_.linear_y_speed * sin(odom_.yaw) * dt_s;
    odom_.y += odom_.linear_y_speed * cos(odom_.yaw) * dt_s + odom_.linear_x_speed * sin(odom_.yaw) * dt_s;

    odom_.yaw += odom_.angular_speed * dt_s;
    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);

    // Serial.printf("odom(%f,%f)\n", odom_.x, odom_.y);
}

void Kinematics::kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed,
                                   float &out_wheel_speed1, float &out_wheel_speed2, float &out_wheel_speed3, float &out_wheel_speed4)
{
    // TODO : 参数化
    const float a = 108.0f;
    const float b = 88.5f;

    out_wheel_speed1 = linear_x_speed - linear_y_speed - angular_speed * (a + b);
    out_wheel_speed2 = linear_x_speed + linear_y_speed + angular_speed * (a + b);
    out_wheel_speed3 = linear_x_speed + linear_y_speed - angular_speed * (a + b);
    out_wheel_speed4 = linear_x_speed - linear_y_speed + angular_speed * (a + b);

    // Serial.printf("out_wheel_speed[%f,%f,%f,%f]\n", out_wheel_speed1, out_wheel_speed2, out_wheel_speed3, out_wheel_speed4);
}

void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed,
                                   float &linear_x_speed, float &linear_y_speed, float &angular_speed)
{
    // TODO : 参数化
    const float a = 108.0f;
    const float b = 88.5f;

    // 计算机器人的 x 轴线速度，公式为四个轮子转速之和的平均值。
    linear_x_speed = (wheel1_speed + wheel2_speed + wheel3_speed + wheel4_speed) / 4.0f;
    linear_y_speed = (-wheel1_speed + wheel2_speed + wheel3_speed - wheel4_speed) / 4.0f;
    angular_speed = float(-wheel1_speed + wheel2_speed - wheel3_speed + wheel4_speed) / (4.0f * (a + b));

    // Serial.printf("angular_speed:%f wheel_speed[%f,%f,%f,%f]\n",angular_speed, wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed);
}

odom_t &Kinematics::odom()
{
    Kinematics::Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    return odom_;
}

float Kinematics::motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}