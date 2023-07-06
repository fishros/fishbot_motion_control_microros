/**
 * @file fishbot_config.h
 * @author fishros (fishros@foxmail.com)
 * @brief 机器人配置汇总
 * @version V1.0.0
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#ifndef __FISHBOT_CONFIG_H__
#define __FISHBOT_CONFIG_H__

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "fishlog.h"

/*=========================================常量值定义========================================*/
#define CONFIG_TRANSPORT_MODE_SERIAL "serial"              // 串口模式，0
#define CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT "udp_client" // UDP客户端模式，1

/*=========================================默认值定义=====================================*/
#define CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP "192.168.4.134" // 默认UDP服务端IP
#define CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT "8888"        // 默认UDP服务端端口号
#define CONFIG_DEFAULT_TRANSPORT_MODE "udp_client"                   // 默认传输模式-udp_client模式
#define CONFIG_DEFAULT_SERIAL_SERIAL_BAUD 921600
#define CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD "921600"

//------------------------------------WIFI SSID-----------------------------------------
#define CONFIG_DEFAULT_WIFI_STA_SSID "oneKM"
#define CONFIG_DEFAULT_WIFI_STA_PSWK "88888888"

//--------------------------------------电机相关配置---------------------------------------
#define CONFIG_DEFAULT_MOTOR_PID_KP "0.625"
#define CONFIG_DEFAULT_MOTOR_PID_KI "0.125"
#define CONFIG_DEFAULT_MOTOR_PID_KD "0.0"

#define CONFIG_DEFAULT_MOTOR_OUT_LIMIT_LOW "-100"
#define CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH "100"
#define CONFIG_DEFAULT_MOTOR_PARAM_REDUCATION_RATIO "45"
#define CONFIG_DEFAULT_MOTOR_PARAM_PULSE_RATION "44"
#define CONFIG_DEFAULT_MOTOR_PARAM_WHEEL_DIAMETER "48"
//-------------------------------------默认轮距----------------------------------------------
#define CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE "150"

//------------------------------------IO相关配置----------------------------------------------
// IMU
#define CONFIG_DEFAULT_IMU_SDA_GPIO 18
#define CONFIG_DEFAULT_IMU_SCL_GPIO 19
// OLED
#define CONFIG_DEFAULT_OLED_SDA_GPIO 18
#define CONFIG_DEFAULT_OLED_SCL_GPIO 19
// 电池电压
#define CONFIG_DEFAULT_BATTERY_MEASURE_GPIO 34
// 超声波
#define CONFIG_DEFAULT_ULTRASONIC_TRIG_GPIO 27
#define CONFIG_DEFAULT_ULTRASONIC_ECHO_GPIO 21
// 舵机相关
#define CONFIG_DEFAULT_SERVP01_GPIO 4
#define CONFIG_DEFAULT_SERVP02_GPIO 5
#define CONFIG_DEFAULT_SERVP03_GPIO 14
#define CONFIG_DEFAULT_SERVP04_GPIO 15
#define CONFIG_DEFAULT_SERVP05_GPIO 16
#define CONFIG_DEFAULT_SERVP06_GPIO 17
// 编码器相关
#define CONFIG_DEFAULT_ENCODER0_A_GPIO 32
#define CONFIG_DEFAULT_ENCODER0_B_GPIO 33
#define CONFIG_DEFAULT_ENCODER1_A_GPIO 26
#define CONFIG_DEFAULT_ENCODER1_B_GPIO 25
#define CONFIG_DEFAULT_PCNT_UTIL_00 0
#define CONFIG_DEFAULT_PCNT_UTIL_01 1
// 电机驱动
#define CONFIG_DEFAULT_MOTOR0_A_GPIO 22
#define CONFIG_DEFAULT_MOTOR0_B_GPIO 23
#define CONFIG_DEFAULT_MOTOR1_A_GPIO 12
#define CONFIG_DEFAULT_MOTOR1_B_GPIO 13

//-----------------------------ROS2节点相关配置-------------------------------------------
#define CONFIG_DEFAULT_ROS2_NODE_NAME "fishbot_motion_control"
#define CONFIG_DEFAULT_ROS2_NAMESPACE ""
#define CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME "odom"
#define CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID "odom"
#define CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME "cmd_vel"
#define CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_PERIOD "50"
#define CONFIG_DEFAULT_ROS2_HANDLE_NUM "2"
#define CONFIG_DEFAULT_ROS2_RCL_SPIN_TIME "100"
#define CONFIG_DEFAULT_ROS2_TIME_SYNC_TIMEOUT "5000"

/*========================================配置名称=======================================*/
#define CONFIG_NAME_NAMESPACE "fishbot"
#define CONFIG_NAME_TRANSPORT_MODE "transport_mode"
#define CONFIG_NAME_TRANSPORT_SERIAL_BAUD "serial_baud"
#define CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_IP "server_ip"
#define CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_PORT "server_port"

// WIFI名称
#define CONFIG_NAME_WIFI_STA_SSID_NAME "wifi_ssid"
#define CONFIG_NAME_WIFI_STA_PSWK_NAME "wifi_pswd"
#define CONFIG_NAME_IS_FIRST_STARTUP "first_startup"

// ROS2相关
#define CONFIG_NAME_ROS2_ODOM_TOPIC_NAME "odom_topic"
#define CONFIG_NAME_ROS2_ODOM_FRAMEID_NAME "odom_frameid"
#define CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME "cmd_vel_topic"
#define CONFIG_NAME_ROS2_NODE_NAME "node_name"
#define CONFIG_NAME_ROS2_ODOM_PUBLISH_TIMER_TIME "odom_period"
#define CONFIG_NAME_ROS2_NAMESPACE ""

// 电机配置
#define CONFIG_NAME_MOTOR_PID_KP "pid_kp"
#define CONFIG_NAME_MOTOR_PID_KI "pid_ki"
#define CONFIG_NAME_MOTOR_PID_KD "pid_kd"
#define CONFIG_NAME_MOTOR_OUT_LIMIT_LOW "pid_out_min"
#define CONFIG_NAME_MOTOR_OUT_LIMIT_HIGH "pid_out_high"

#define CONFIG_NAME_MOTOR0_PARAM_REDUCATION_RATIO "motor0_reducation"
#define CONFIG_NAME_MOTOR0_PARAM_PULSE_RATION "motor0_pulse"
#define CONFIG_NAME_MOTOR0_PARAM_WHEEL_DIAMETER "motor0_wheel_iameter"

#define CONFIG_NAME_MOTOR1_PARAM_REDUCATION_RATIO "motor1_reducation"
#define CONFIG_NAME_MOTOR1_PARAM_PULSE_RATION "motor1_pulse"
#define CONFIG_NAME_MOTOR1_PARAM_WHEEL_DIAMETER "motor1_wheel_diameter"
#define CONFIG_NAME_KINEMATIC_WHEEL_DISTANCE "wheel_dist"

#define FIRST_START_TIP "=================================================\n     wwww.fishros.com        \nfishbot-motion-control-v1.0.0\n=================================================\n"

typedef enum
{
    CONFIG_PARSE_ERROR = -1,
    CONFIG_PARSE_NODATA = 0,
    CONFIG_PARSE_OK = 1,
} fishbot_config_status;

class FishBotConfig
{
private:
    /* data */
    Preferences preferences;

public:
    void init(String namespace_);
    uint32_t is_first_startup();

    bool config(String key, String value);
    // bool config(String key, int32_t value);
    // bool config(String key, uint32_t value);
    // bool config(String key, const float_t value);
    // bool config(String key, const bool value);

    String config_str();
    String board_name();
    // 基础配置
    uint32_t serial_baudrate();
    String wifi_sta_ssid();
    String wifi_sta_pswd();
    String wifi_ap_ssid();
    String wifi_ap_pswd();
    // MicroROS相关
    String microros_transport_mode();
    String microros_uclient_server_ip();
    uint32_t microros_uclient_server_port();
    // ROS2相关
    String ros2_nodename();
    String ros2_namespace();
    String ros2_odom_topic_name();
    String ros2_odom_frameid();
    String ros2_twist_topic_name();
    uint32_t odom_publish_period();
    // 运动学相关配置
    uint32_t kinematics_reducation_ration();
    uint32_t kinematics_pulse_ration();
    uint32_t kinematics_wheel_diameter();
    float kinematics_wheel_distance();
    float kinematics_pid_kp();
    float kinematics_pid_ki();
    float kinematics_pid_kd();
    float kinematics_pid_out_limit();

    int8_t loop_config_uart(int c, char result[][32]);
    int8_t split_str(const char *line, char result[][32]);

    FishBotConfig(/* args */) = default;
    ~FishBotConfig() = default;
};

#endif // __FISHBOT_CONFIG_H__