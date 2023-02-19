#include "transport_microros.h"

/*==================MicroROS消息============================*/
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
char odom_frame_id[16];
fishbot_interfaces__srv__FishBotConfig_Response config_res;
fishbot_interfaces__srv__FishBotConfig_Request config_req;
static micro_ros_utilities_memory_conf_t conf = {0};
static int keyvalue_capacity = 100;

/*==================MicroROS订阅发布者服务========================*/
rcl_publisher_t odom_publisher;
rcl_subscription_t twist_subscriber;
rcl_service_t config_service;
rcl_wait_set_t wait_set;

/*==================MicroROS相关执行器&节点===================*/
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


bool create_fishbot_transport()
{
    String nodename = config.ros2_nodename();
    String ros2namespace = config.ros2_namespace();
    String twist_topic = config.ros2_twist_topic_name();
    String odom_topic = config.ros2_odom_topic_name();
    String odom_frameid_str = config.ros2_odom_frameid();
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, odom_frameid_str.c_str());
    const unsigned int timer_timeout = config.odom_publish_period();
    delay(500);
    allocator = rcl_get_default_allocator();
    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCSOFTCHECK(rclc_node_init_default(&node, nodename.c_str(), ros2namespace.c_str(), &support));
    RCSOFTCHECK(rclc_publisher_init_best_effort(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        odom_topic.c_str()));
    RCSOFTCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        twist_topic.c_str()));
    RCSOFTCHECK(rclc_service_init_default(
        &config_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(fishbot_interfaces, srv, FishBotConfig),
        "/fishbot_config"));
    RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), callback_odom_publisher_timer_));
    RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &callback_twist_subscription_, ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
    RCSOFTCHECK(rclc_executor_add_service(&executor, &config_service, &config_req, &config_res, callback_config_service_));
    return true;
}