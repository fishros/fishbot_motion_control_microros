/**
 * @file fishbot_config.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief 机器人配置处理类
 * @version V1.0.0
 * @date 2023-01-04
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#include "fishbot_config.h"

void FishBotConfig::init(String namespace_)
{
    preferences.begin(CONFIG_NAME_NAMESPACE);
    if (is_first_startup())
    {
        fishlog_debug("config", "config default setting.");

        preferences.putString("serial_baud", CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD);
        preferences.putString("wifi_ssid", CONFIG_DEFAULT_WIFI_STA_SSID);
        preferences.putString("wifi_pswd", CONFIG_DEFAULT_WIFI_STA_PSWK);

        preferences.putString("microros_mode", CONFIG_DEFAULT_TRANSPORT_MODE);
        preferences.putString("udpserver_ip", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP);
        preferences.putString("udpserver_port", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT);

        preferences.putString("ros2_nodename", CONFIG_DEFAULT_ROS2_NODE_NAME);
        preferences.putString("ros2_namespace", CONFIG_DEFAULT_ROS2_NAMESPACE);
        preferences.putString("odom_topic", CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
        preferences.putString("odom_frameid", CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID);
        preferences.putString("odom_pub_period", CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_PERIOD);
        preferences.putString("twist_topic", CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME);

        preferences.putString("wheel_distance", CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE);
        preferences.putString("reducate_ration", CONFIG_DEFAULT_MOTOR_PARAM_REDUCATION_RATIO);
        preferences.putString("pulse_ration", CONFIG_DEFAULT_MOTOR_PARAM_PULSE_RATION);
        preferences.putString("wheel_diameter", CONFIG_DEFAULT_MOTOR_PARAM_WHEEL_DIAMETER);

        preferences.putString("pid_kp", CONFIG_DEFAULT_MOTOR_PID_KP);
        preferences.putString("pid_ki", CONFIG_DEFAULT_MOTOR_PID_KI);
        preferences.putString("pid_kd", CONFIG_DEFAULT_MOTOR_PID_KD);
        preferences.putString("pid_outlimit", CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH);

        preferences.putBool("first_startup", false);
    }
}

uint32_t FishBotConfig::is_first_startup()
{
    fishlog_debug("config", "first_startup=%d", preferences.getBool("first_startup", true) ? 1 : 0);
    return preferences.getBool("first_startup", true);
}

bool FishBotConfig::config(String key, String value)
{
    fishlog_debug("config", "save config key=%s,value=%s", key.c_str(), value.c_str());
    return preferences.putString(key.c_str(), value.c_str());
}
// bool FishBotConfig::config(String key, int32_t value)
// {
//     return preferences.putInt(key.c_str(), value);
// }
// bool FishBotConfig::config(String key, uint32_t value)
// {
//     return preferences.putUInt(key.c_str(), value);
// }
// bool FishBotConfig::config(String key, const float_t value)
// {
//     return preferences.putFloat(key.c_str(), value);
// }
// bool FishBotConfig::config(String key, const bool value)
// {
//     return preferences.putBool(key.c_str(), value);
// }
/*

        preferences.putString("serial_baud", CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD);
        preferences.putString("wifi_ssid", CONFIG_DEFAULT_WIFI_STA_SSID);
        preferences.putString("wifi_pswd", CONFIG_DEFAULT_WIFI_STA_PSWK);

        preferences.putString("microros_mode", CONFIG_DEFAULT_TRANSPORT_MODE);
        preferences.putString("udpserver_ip", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP);
        preferences.putString("udpserver_port", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT);

        preferences.putString("ros2_nodename", CONFIG_DEFAULT_ROS2_NODE_NAME);
        preferences.putString("ros2_namespace", CONFIG_DEFAULT_ROS2_NAMESPACE);
        preferences.putString("odom_topic", CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
        preferences.putString("odom_frameid", CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID);
        preferences.putString("odom_pub_period", CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_PERIOD);
        preferences.putString("twist_topic", CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME);

        preferences.putString("wheel_distance", CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE);
        preferences.putString("reducate_ration", CONFIG_DEFAULT_MOTOR0_PARAM_REDUCATION_RATIO);
        preferences.putString("pulse_ration", CONFIG_DEFAULT_MOTOR0_PARAM_PULSE_RATION);
        preferences.putString("wheel_diameter", CONFIG_DEFAULT_MOTOR0_PARAM_WHEEL_DIAMETER);

        preferences.putString("pid_kp", CONFIG_DEFAULT_MOTOR_PID_KP);
        preferences.putString("pid_ki", CONFIG_DEFAULT_MOTOR_PID_KI);
        preferences.putString("pid_kd", CONFIG_DEFAULT_MOTOR_PID_KD);
        preferences.putString("pid_outlimit", CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH);

        preferences.putBool("first_startup", false);

*/

String FishBotConfig::config_str()
{
    String config("");
    config.concat("$first_startup=");
    config.concat(is_first_startup());
    config.concat("\n$serial_baud=");
    config.concat(serial_baudrate());
    config.concat("\n$wifi_ssid=");
    config.concat(wifi_sta_ssid());
    config.concat("\n$wifi_pswd=");
    config.concat(wifi_sta_pswd());

    config.concat("\n$wifi_ap_ssid=");
    config.concat(wifi_ap_ssid());

    config.concat("\n$wifi_ap_pswd=");
    config.concat(wifi_ap_pswd());

    config.concat("\n$microros_mode=");
    config.concat(microros_transport_mode());

    config.concat("\n$udpserver_ip=");
    config.concat(microros_uclient_server_ip());

    config.concat("\n$udpserver_port=");
    config.concat(microros_uclient_server_port());

    config.concat("\n$ros2_nodename=");
    config.concat(ros2_nodename());

    config.concat("\n$ros2_namespace=");
    config.concat(ros2_namespace());

    config.concat("\n$odom_topic=");
    config.concat(ros2_odom_topic_name());

    config.concat("\n$odom_frameid=");
    config.concat(ros2_odom_frameid());

    config.concat("\n$twist_topic=");
    config.concat(ros2_twist_topic_name());

    config.concat("\n$odom_pub_period=");
    config.concat(odom_publish_period());

    config.concat("\n$reducate_ration=");
    config.concat(kinematics_reducation_ration());

    config.concat("\n$pulse_ration=");
    config.concat(kinematics_pulse_ration());

    config.concat("\n$wheel_diameter=");
    config.concat(kinematics_wheel_diameter());

    config.concat("\n$wheel_distance=");
    config.concat(kinematics_wheel_distance());

    config.concat("\n$pid_kp=");
    config.concat(kinematics_pid_kp());

    config.concat("\n$pid_ki=");
    config.concat(kinematics_pid_ki());

    config.concat("\n$pid_kd=");
    config.concat(kinematics_pid_kd());

    config.concat("\n$pid_outlimit=");
    config.concat(kinematics_pid_out_limit());

    config.concat("\n$board=motion_board");
    config.concat("\n$version=v1.1.0\n");

    return config;
}

String FishBotConfig::board_name()
{
    char board_name[16];
    uint8_t macAddr[6];
    WiFi.macAddress(macAddr);
    sprintf(board_name, "FISHBOT_%02X%02X", macAddr[4], macAddr[5]);
    return String(board_name);
}

uint32_t FishBotConfig::serial_baudrate()
{
    return preferences.getString(CONFIG_NAME_TRANSPORT_SERIAL_BAUD, CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD).toInt();
}

String FishBotConfig::wifi_sta_ssid()
{
    return preferences.getString(CONFIG_NAME_WIFI_STA_SSID_NAME, CONFIG_DEFAULT_WIFI_STA_SSID);
}

String FishBotConfig::wifi_sta_pswd()
{

    return preferences.getString(CONFIG_NAME_WIFI_STA_PSWK_NAME, CONFIG_DEFAULT_WIFI_STA_PSWK);
}

String FishBotConfig::wifi_ap_ssid()
{
    return board_name();
}
String FishBotConfig::wifi_ap_pswd()
{
    return "";
}
// MicroROS相关
String FishBotConfig::microros_transport_mode()
{
    return preferences.getString("microros_mode", CONFIG_DEFAULT_TRANSPORT_MODE);
}
String FishBotConfig::microros_uclient_server_ip()
{
    return preferences.getString("udpserver_ip", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP);
}
uint32_t FishBotConfig::microros_uclient_server_port()
{
    return preferences.getString("udpserver_port", CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT).toInt();
}
// ROS2相关
String FishBotConfig::ros2_nodename()
{
    return preferences.getString("ros2_nodename", CONFIG_DEFAULT_ROS2_NODE_NAME);
}
String FishBotConfig::ros2_namespace()
{
    return preferences.getString("ros2_namespace", CONFIG_DEFAULT_ROS2_NAMESPACE);
}
String FishBotConfig::ros2_odom_topic_name()
{
    return preferences.getString("odom_topic", CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
}
String FishBotConfig::ros2_odom_frameid()
{
    return preferences.getString("odom_frameid", CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID);
}
uint32_t FishBotConfig::odom_publish_period()
{
    return preferences.getString("odom_pub_period", CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_PERIOD).toInt();
}
String FishBotConfig::ros2_twist_topic_name()
{
    return preferences.getString("twist_topic", CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME);
}
// 运动学相关配置
float FishBotConfig::kinematics_wheel_distance()
{
    return preferences.getString("wheel_distance", CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE).toFloat();
}
uint32_t FishBotConfig::kinematics_reducation_ration()
{
    return preferences.getString("reducate_ration", CONFIG_DEFAULT_MOTOR_PARAM_REDUCATION_RATIO).toInt();
}
uint32_t FishBotConfig::kinematics_pulse_ration()
{
    return preferences.getString("pulse_ration", CONFIG_DEFAULT_MOTOR_PARAM_PULSE_RATION).toInt();
}
uint32_t FishBotConfig::kinematics_wheel_diameter()
{
    return preferences.getString("wheel_diameter", CONFIG_DEFAULT_MOTOR_PARAM_WHEEL_DIAMETER).toInt();
}
float FishBotConfig::kinematics_pid_kp()
{
    return preferences.getString("pid_kp", CONFIG_DEFAULT_MOTOR_PID_KP).toFloat();
}
float FishBotConfig::kinematics_pid_ki()
{
    return preferences.getString("pid_ki", CONFIG_DEFAULT_MOTOR_PID_KI).toFloat();
}
float FishBotConfig::kinematics_pid_kd()
{
    return preferences.getString("pid_kd", CONFIG_DEFAULT_MOTOR_PID_KD).toFloat();
}
float FishBotConfig::kinematics_pid_out_limit()
{
    return preferences.getString("pid_outlimit", CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH).toInt();
}

/**
 * @brief
 * @param line
 * @param result  1 正常分割，-1 错误分割
 * @return int8_t
 */
int8_t FishBotConfig::split_str(const char *line, char result[][32])
{
    if (line[0] != '$')
        return CONFIG_PARSE_ERROR;
    uint16_t index = 0;
    uint16_t count = 0;
    uint16_t temp_index = 0;
    for (index = 1; line[index] != '\0'; index++)
    {
        if (line[index] == '=')
        {
            result[count++][temp_index++] = '\0';
            temp_index = 0;
            continue;
        }
        result[count][temp_index++] = line[index];
    }
    result[count][temp_index++] = '\0';

    if (count != 1)
    {
        return CONFIG_PARSE_ERROR;
    }
    return CONFIG_PARSE_OK;
}

/**
 * @brief
 *
 * @param c
 * @param result  0无数据，1有配置，-1错误解析
 * @return int8_t
 */
int8_t FishBotConfig::loop_config_uart(int c, char result[][32])
{
    static char line[512];
    static int index = 0;
    if (c == '\n')
    {
        line[index] = '\0';
        index = 0;
        return split_str(line, result);
    }
    else if (c > 0 && c < 127)
    {
        line[index] = c;
        ++index;
    }
    return CONFIG_PARSE_NODATA;
}
