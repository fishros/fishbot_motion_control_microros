#ifndef __FISHBOT_H__
#define __FISHBOT_H__

#include <Arduino.h>

typedef enum {
    TRANSPORT_MQTT=1,
    TRANSPORT_MICROROS,
}transport_mode_t;

transport_mode_t mode;

void transport_init();
// void publish_odom_data();
// void publish_imu_data();
void subscribe_twist();

int64_t transport_timestamp();

#endif