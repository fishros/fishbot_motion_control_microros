#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <TimeLib.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

rclc_support_t support;
rcl_allocator_t allocator;

#define HWSERIAL Serial

#define LED_PIN 13

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }

const int timeout_ms = 1000;
static int64_t time_ms;
static time_t time_seconds;
char time_str[25];

void error_loop()
{
    while (1)
    {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void setup()
{

    HWSERIAL.begin(115200); // Configure debug serial
    IPAddress agent_ip(192, 168, 2, 105);
    size_t agent_port = 8888;
    char ssid[] = "m3";
    char psk[] = "88888888";
    set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

    delay(2000);

    allocator = rcl_get_default_allocator();

    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
}

void loop()
{
    // Synchronize time
    RCCHECK(rmw_uros_sync_session(timeout_ms));

    while (true)
    {
        /* code */
        time_ms = rmw_uros_epoch_millis();
        Serial.println(rmw_uros_epoch_millis());
        time_seconds = time_ms / 1000;
        setTime(time_seconds);
        sprintf(time_str, "%02d.%02d.%04d %02d:%02d:%02d.%03d", day(), month(), year(), hour(), minute(), second(), (uint)time_ms % 1000);
        HWSERIAL.print("Agent date: ");
        HWSERIAL.println(time_str);
        delay(10);
    }

    // if (time_ms > 0)
    // {
    //   time_seconds = time_ms / 1000;
    //   setTime(time_seconds);
    //   sprintf(time_str, "%02d.%02d.%04d %02d:%02d:%02d.%03d", day(), month(), year(), hour(), minute(), second(), (uint)time_ms % 1000);

    //   HWSERIAL.print("Agent date: ");
    //   HWSERIAL.println(time_str);
    // }
    // else
    // {
    //   HWSERIAL.print("Session sync failed, error code: ");
    //   HWSERIAL.println((int)time_ms);
    // }

    // delay(1001);
}