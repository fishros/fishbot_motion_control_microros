/**
 * @file Kinematics.h
 * @author fishros@foxmail.com
 * @brief 机器人模型设置,编码器轮速转换,ODOM推算,线速度角速度分解
 * @version V1.0.0
 * @date 2022-12-10
 *
 * @copyright Copyright www.fishros.com (c) 2022
 *
 */
#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__
#include <Arduino.h>
// #include <fishlog.h>

/**
 * @brief 电机相关结构体
 * @param  id   uint8_t  电机编号
 * @param  reducation_ratio uint16_t   减速器减速比，轮子转一圈，电机需要转的圈数
 * @param  pulse_ration uint16_t       脉冲比，电机转一圈所产生的脉冲数
 * @param  wheel_diameter float      轮子的外直径，单位mm
 * @param  per_pulse_distance float    无需配置，单个脉冲轮子前进的距离，单位mm，设置时自动计算
                               ,单个脉冲距离=轮子转一圈所行进的距离/轮子转一圈所产生的脉冲数
                               ,per_pulse_distance= (wheel_diameter*3.1415926)/(pulse_ration*reducation_ratio)
 * @param  speed_factor uint16_t 无需配置，计算速度时使用的速度因子，设置时自动计算，speed_factor计算方式如下
 *                             ,设 dt（单位us,1s=1000ms=10^6us）时间内的脉冲数为dtick
 *                             ,速度speed = per_pulse_distance*dtick/(dt/1000/1000)=(per_pulse_distance*1000*1000)*dtick/dt
 *                             ,记 speed_factor = (per_pulse_distance*1000*1000)
 * @param  motor_speed int16_t 无需配置，当前电机速度，计算时使用
 * @param  last_encoder_tick int64_t 无需配置，上次电机的编码器读数
 * @param last_update_time uint64_t 无需配置，上次更新数据的时间，单位us
 */
typedef struct
{
    uint8_t id;                // 电机编号
    uint16_t reducation_ratio; // 减速器减速比，轮子转一圈，电机需要转的圈数
    uint16_t pulse_ration;     // 脉冲比，电机转一圈所产生的脉冲数
    float wheel_diameter;      // 轮子的外直径，单位mm

    float per_pulse_distance;  // 无需配置，单个脉冲轮子前进的距离，单位mm，设置时自动计算
                               // 单个脉冲距离=轮子转一圈所行进的距离/轮子转一圈所产生的脉冲数
                               // per_pulse_distance= (wheel_diameter*3.1415926)/(pulse_ration*reducation_ratio)
    uint32_t speed_factor;     // 无需配置，计算速度时使用的速度因子，设置时自动计算，speed_factor计算方式如下
                               // 设 dt（单位us,1s=1000ms=10^6us）时间内的脉冲数为dtick
                               // 速度speed = per_pulse_distance*dtick/(dt/1000/1000)=(per_pulse_distance*1000*1000)*dtic/dt
                               // 记 speed_factor = (per_pulse_distance*1000*1000)
    int16_t motor_speed;       // 无需配置，当前电机速度mm/s，计算时使用
    int64_t last_encoder_tick; // 无需配置，上次电机的编码器读数
    uint64_t last_update_time; // 无需配置，上次更新数据的时间，单位us
} motor_param_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

/**
 * @brief 里程计相关信息，根据轮子速度信息和运动模型推算而来
 *
 */
typedef struct
{
    float x;                 // 坐标x
    float y;                 // 坐标y
    float yaw;               // yaw
    quaternion_t quaternion; // 姿态四元数
    float linear_x_speed;      // x线速度
    float linear_y_speed;      // y线速度
    float angular_speed;     // 角速度
} odom_t;

class Kinematics
{
private:
    motor_param_t motor_param_[4];
    odom_t odom_;          // 里程计数据
    float wheel_distance_; // 轮子间距
public:
    Kinematics(/* args */) = default;
    ~Kinematics() = default;

    static void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q);
    static void TransAngleInPI(float angle, float &out_angle);

    void set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter);
    void set_kinematic_param(float wheel_distance);

    void kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed, 
            float &out_wheel1_speed, float &out_wheel2_speed, float &out_wheel3_speed, float &out_wheel4_speed);
    void kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed, 
                                    float &linear_x_speed,float &linear_y_speed, float &angular_speed);
    void update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2, int32_t motor_tick3, int32_t motor_tick4);

    odom_t &odom();
    float motor_speed(uint8_t id);

private:
    void update_bot_odom_(uint32_t dt);
};

#endif // __KINEMATICS_H__