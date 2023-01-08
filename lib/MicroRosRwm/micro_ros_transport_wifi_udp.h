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

static bool set_microros_wifi_transports(const char *ssid, const char *pswd, IPAddress agent_ip, uint16_t agent_port, String device_name)
{
    if (!WiFi.setHostname(device_name.c_str()))
    {
        Serial.println("Hostname failed to configure");
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pswd);
    WiFi.useStaticBuffers(true);
    WiFi.setAutoReconnect(true);
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

    WiFi.setSleep(WIFI_PS_NONE);

    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *)&locator,
        platformio_transport_open_wifi_udp,
        platformio_transport_close_wifi_udp,
        platformio_transport_write_wifi_udp,
        platformio_transport_read_wifi_udp);
    return true;
}