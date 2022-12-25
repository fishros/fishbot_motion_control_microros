#include "fishbot.h"

/*==================MicroROS消息============================*/
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;
char odom_frame_id[16];
fishbot_interfaces__srv__FishBotConfig_Response config_res;
fishbot_interfaces__srv__FishBotConfig_Request config_req;
static micro_ros_utilities_memory_conf_t conf = {0};
static char config_srv_memory[512];
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

/*========================FishBot控制相关====================*/
// Oled oled;
PidController pid_controller[2];
Esp32PcntEncoder encoders[2];
Esp32McpwmMotor motor;
Kinematics kinematics;
FishBotConfig config;

bool setup_fishbot()
{
    // 1.初始化
    config.init(CONFIG_NAME_NAMESPACE);
    Serial.begin(CONFIG_DEFAULT_SERIAL_SERIAL_BAUD);
    Serial.println(FIRST_START_TIP);
    Serial.println(config.config_str());
    // 2.设置IO 电机&编码器
    motor.attachMotors(CONFIG_DEFAULT_MOTOR0_A_GPIO, CONFIG_DEFAULT_MOTOR0_B_GPIO, CONFIG_DEFAULT_MOTOR1_A_GPIO, CONFIG_DEFAULT_MOTOR1_B_GPIO);
    encoders[0].init(CONFIG_DEFAULT_ENCODER0_A_GPIO, CONFIG_DEFAULT_ENCODER0_B_GPIO, CONFIG_DEFAULT_PCNT_UTIL_00);
    encoders[1].init(CONFIG_DEFAULT_ENCODER1_A_GPIO, CONFIG_DEFAULT_ENCODER1_B_GPIO, CONFIG_DEFAULT_PCNT_UTIL_01);
    // 3.设置PID
    pid_controller[0].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
    pid_controller[1].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
    pid_controller[0].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    pid_controller[1].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    // 4.设置运动学参数
    kinematics.set_motor_param(0, config.kinematics_reducation_ration(), config.kinematics_pulse_ration(), config.kinematics_wheel_diameter());
    kinematics.set_motor_param(1, config.kinematics_reducation_ration(), config.kinematics_pulse_ration(), config.kinematics_wheel_diameter());
    kinematics.set_kinematic_param(config.kinematics_wheel_distance());
    // 5.设置microRos
    microros_setup_transport_udp_client_();
    // 6.添加/cmd_vel话题订阅
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &callback_twist_subscription_, ON_NEW_DATA));
    RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
    RCSOFTCHECK(rclc_executor_add_service(&executor, &config_service, &config_req, &config_res, callback_config_service_));
    return true;
}

void loop_fishbot_control()
{
    static float out_motor_speed[2];
    kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks());
    out_motor_speed[0] = pid_controller[0].update(kinematics.motor_speed(0));
    out_motor_speed[1] = pid_controller[1].update(kinematics.motor_speed(1));
    motor.updateMotorSpeed(0, out_motor_speed[0]);
    motor.updateMotorSpeed(1, out_motor_speed[1]);
}

void loop_fishbot_transport()
{
    if (!rmw_uros_epoch_synchronized())
    {
        RCSOFTCHECK(rmw_uros_sync_session(1000));
        Serial.print("current_time:");
        Serial.println(rmw_uros_epoch_millis());
        delay(10);
        return;
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

bool microros_setup_transport_udp_client_()
{
    String ssid = config.wifi_sta_ssid();
    String ip = config.microros_uclient_server_ip();
    String password = config.wifi_sta_pswd();
    uint32_t agent_port = config.microros_uclient_server_port();

    String nodename = config.ros2_nodename();
    String ros2namespace = config.ros2_namespace();
    String twist_topic = config.ros2_twist_topic_name();
    String odom_topic = config.ros2_odom_topic_name();
    String odom_frameid_str = config.ros2_odom_frameid();
    sprintf(odom_frame_id, "%s", odom_frameid_str.c_str());
    odom_msg.header.frame_id.data = odom_frame_id;
    const unsigned int timer_timeout = config.odom_publish_period();
    // 初始化数据接收配置
    config_req.key.capacity = keyvalue_capacity;
    config_req.key.data = config_srv_memory;
    config_req.key.size = 0;
    config_req.value.capacity = keyvalue_capacity;
    config_req.value.data = config_srv_memory + keyvalue_capacity;
    config_req.value.size = 0;

    config_res.key.capacity = keyvalue_capacity;
    config_res.key.data = config_srv_memory + keyvalue_capacity * 2;
    config_res.key.size = 0;
    config_res.value.capacity = keyvalue_capacity;
    config_res.value.data = config_srv_memory + keyvalue_capacity * 3;
    config_res.value.size = 0;

    IPAddress agent_ip;
    agent_ip.fromString(ip);
    set_microros_wifi_transports((char *)ssid.c_str(), (char *)password.c_str(), agent_ip, agent_port);
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
    return true;
}

bool microros_setup_transport_serial_()
{
    return true;
}

void callback_odom_publisher_timer_(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        odom_t odom = kinematics.odom();
        int64_t stamp = rmw_uros_epoch_millis();
        odom_msg.header.stamp.sec = stamp * 1e-3;
        odom_msg.header.stamp.nanosec = stamp - odom_msg.header.stamp.sec * 1000;
        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.orientation.w = odom.quaternion.w;
        odom_msg.pose.pose.orientation.x = odom.quaternion.x;
        odom_msg.pose.pose.orientation.y = odom.quaternion.y;
        odom_msg.pose.pose.orientation.z = odom.quaternion.z;
        odom_msg.twist.twist.angular.z = odom.angular_speed;
        odom_msg.twist.twist.linear.x = odom.linear_speed;

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
}

void callback_twist_subscription_(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    static float target_motor_speed1, target_motor_speed2;
    kinematics.kinematic_inverse(msg->linear.x * 1000, msg->angular.z, target_motor_speed1, target_motor_speed2);
    pid_controller[0].update_target(target_motor_speed1);
    pid_controller[1].update_target(target_motor_speed2);
}

void callback_config_service_(const void *req, void *res)
{
    fishbot_interfaces__srv__FishBotConfig_Request *req_in = (fishbot_interfaces__srv__FishBotConfig_Request *)req;
    fishbot_interfaces__srv__FishBotConfig_Response *res_in = (fishbot_interfaces__srv__FishBotConfig_Response *)res;
    Serial.print("===================================================\n");
    String recv_key(req_in->key.data);
    String recv_value(req_in->value.data);
    Serial.printf("recv service key=%s,value=%s\n", req_in->key.data, req_in->value.data);
    config.config(recv_key, recv_value);
    Serial.print("---------------------------------------------------\n");
    Serial.println(config.config_str());
    if (recv_key.startsWith("pid_"))
    {
        pid_controller[0].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
        pid_controller[1].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
        pid_controller[0].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
        pid_controller[1].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    }
    Serial.print("===================================================\n");
}