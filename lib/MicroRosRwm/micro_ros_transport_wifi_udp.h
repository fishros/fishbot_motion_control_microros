/*
 * @Author: lalala123 1054060225@qq.com
 * @Date: 2023-03-23 17:23:07
 * @LastEditors: lalala123 1054060225@qq.com
 * @LastEditTime: 2023-04-02 00:29:42
 * @FilePath: \fishbot_motion_control_microros\lib\MicroRosRwm\micro_ros_transport_wifi_udp.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "micro_ros_platformio.h"

extern "C"
{
    bool platformio_transport_open_wifi_udp(struct uxrCustomTransport *transport);
    bool platformio_transport_close_wifi_udp(struct uxrCustomTransport *transport);
    size_t platformio_transport_write_wifi_udp(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
    size_t platformio_transport_read_wifi_udp(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
}

struct micro_ros_agent_locator
{
    IPAddress address;
    int port;
};
// 用于设置 Micro-ROS 的 WiFi 通信传输方式的函数。
// 具体实现如下：传入 WiFi 的 SSID、密码、代理 IP 地址、代理端口号和设备名称。
static bool set_microros_wifi_transports(const char *ssid, const char *pswd, IPAddress agent_ip, uint16_t agent_port, String device_name)
{
    if (!WiFi.setHostname(device_name.c_str()))
    {
        Serial.println("Hostname failed to configure");
    }
    // 以 Station 模式连接到 WiFi 网络
    WiFi.mode(WIFI_STA);
    // 连接到指定的 WiFi 网络
    WiFi.begin(ssid, pswd);
    // 开启静态缓冲区，以便更好地管理内存
    WiFi.useStaticBuffers(true);
    // 开启自动重连功能
    WiFi.setAutoReconnect(true);

    // 进行 WiFi 连接，最多尝试 4 次，每次间隔 500 毫秒。如果 WiFi 连接失败，则输出错误信息
    for (int i = 0; i < 4; i++)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
        }
        else
        {
            break;
        }
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wifi connected failed!");
    }
    // 不使用睡眠模式
    WiFi.setSleep(WIFI_PS_NONE);
    // 创建一个结构体类型的变量 locator，用于存储代理 IP 地址和端口号
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;
    // 调用 rmw_uros_set_custom_transport 函数，将自定义的 WiFi 传输函数注册到 Micro-ROS 中
    rmw_uros_set_custom_transport(
        false,
        (void *)&locator,
        platformio_transport_open_wifi_udp,
        platformio_transport_close_wifi_udp,
        platformio_transport_write_wifi_udp,
        platformio_transport_read_wifi_udp);
    return true;
}