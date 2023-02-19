
/* MicroROS */
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <fishbot_interfaces/srv/fish_bot_config.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include "micro_ros_transport_serial.h"
#include "micro_ros_transport_wifi_udp.h"


#define RCSOFTCHECK(fn)                                                                           \
    {                                                                                             \
        rcl_ret_t temp_rc = fn;                                                                   \
        if ((temp_rc != RCL_RET_OK))                                                              \
        {                                                                                         \
            fishlog_debug("ros2",                                                                 \
                          "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                         \
    }