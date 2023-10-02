/**
 * @file fishbot.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief 核心文件，硬件控制以及通讯控制
 * @version V1.0.0
 * @date 2023-01-04
 *
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 *
 */
#include "fishbot.h"

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

/*========================FishBot控制相关====================*/
PidController pid_controller[4];
Esp32PcntEncoder encoders[4];
Esp32McpwmMotor motor;
Kinematics kinematics;
FishBotConfig config;
FishBotDisplay display;
BluetoothSerial SerialBT;
float battery_voltage;
OneButton button(0, true);

void WiFiEventCB(WiFiEvent_t event)
{
    Serial.println("WIFI EVENT!");
    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
        display.updateWIFIIp(WiFi.localIP().toString());
        break;
    case SYSTEM_EVENT_STA_LOST_IP:
        display.updateWIFIIp("wait connect!");
        break;
    };
}

void doubleClick()
{
    fishlog_debug("key", "doubleClick() detected.");
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
    {
        config.config("microros_mode", "serial");
    }
    else
    {
        config.config("microros_mode", "udp_client");
    }
    esp_restart();
}

bool setup_fishbot()
{
    // 1.初始化
    Serial.begin(115200);
    fishlog_set_target(Serial);
    config.init(CONFIG_NAME_NAMESPACE);
    Serial.println(FIRST_START_TIP);
    Serial.println(config.config_str());
    display.init();
    display.updateTransMode(config.microros_transport_mode());
    display.updateStartupInfo();
    button.attachDoubleClick(doubleClick);
    // 2.设置IO 电机&编码器
    motor.attachMotor(0, 12, 22);
    motor.attachMotor(1, 16, 17);
    motor.attachMotor(2, 25, 33);
    motor.attachMotor(3, 26, 27);

    encoders[0].init(0, 14, 23);
    encoders[1].init(1, 15, 13);
    encoders[2].init(2, 36, 39);
    encoders[3].init(3, 32, 35);

    // 3.设置PID
    for (int i = 0; i < 4; i++)
    {
        pid_controller[i].update_target(0.0);
        pid_controller[i].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
        pid_controller[i].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
        kinematics.set_motor_param(i, config.kinematics_reducation_ration(), config.kinematics_pulse_ration(), config.kinematics_wheel_diameter());
    }
    // 4.设置运动学参数
    kinematics.set_kinematic_param(config.kinematics_wheel_distance());
    // 7.设置电压测量引脚
    pinMode(34, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    battery_voltage = 5.02 * ((float)analogReadMilliVolts(34) * 1e-3);
    return true;
}

static void deal_command(char key[32], char value[32])
{
    if (strcmp(key, "command") == 0)
    {
        if (strcmp(value, "restart") == 0)
        {
            esp_restart();
        }
        else if (strcmp(value, "read_config") == 0)
        {
            Serial.print(config.config_str());
        }
        return;
    }
    else
    {
        String recv_key(key);
        String recv_value(value);
        config.config(recv_key, recv_value);
        Serial.print("$result=ok\n");
    }
}

bool setup_fishbot_transport()
{
    bool setup_success = true;
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
    {
        fishlog_set_target(Serial);
        WiFi.onEvent(WiFiEventCB);
        setup_success = microros_setup_transport_udp_client_();
        display.updateTransMode("udp_client");
    }
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_SERIAL)
    {
        SerialBT.begin(config.board_name());
        fishlog_set_target(SerialBT);
        microros_setup_transport_serial_();
        display.updateTransMode("serial");
    }

    return true;
}

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

bool destory_fishbot_transport()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_service_fini(&config_service, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    rclc_support_fini(&support);
    return true;
}

void loop_fishbot_control()
{
    static float out_motor_speed[4];
    static uint64_t last_update_info_time = millis();
    static uint8_t index = 0;
    kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks(), encoders[2].getTicks(), encoders[3].getTicks());
    for (index = 0; index < 4; index++)
    {
        // 目标速度为0时停止控制，解决 #https://fishros.org.cn/forum/topic/1372 问题
        if (pid_controller[index].target_ == 0)
        {
            out_motor_speed[index] = 0;
        }
        else
        {
            // 使用 pid_controller 控制器对电机速度进行 PID 控制
            out_motor_speed[index] = pid_controller[index].update(kinematics.motor_speed(index));
        }
        // 将 PID 控制器的输出值作为电机的目标速度进行控制
        motor.updateMotorSpeed(index, out_motor_speed[index]);
        // fishlog_debug("pid", "index:%d target:%f current:%f out=%f", index, pid_controller[index].target_, kinematics.motor_speed(index), out_motor_speed[index]);
    }

    // 电量信息
    if (out_motor_speed[0] == 0 && out_motor_speed[1] == 0)
    {
        battery_voltage = 5.02 * ((float)analogReadMilliVolts(34) * 1e-3);
        display.updateBatteryInfo(battery_voltage);
    }
    // 更新系统信息
    display.updateCurrentTime(rmw_uros_epoch_millis());
    display.updateDisplay();
    button.tick();
}

void loop_fishbot_transport()
{
    static char result[10][32];
    static int config_result;
    while (Serial.available())
    {
        int c = Serial.read();
        config_result = config.loop_config_uart(c, result);
        if (config_result == CONFIG_PARSE_OK)
        {
            deal_command(result[0], result[1]);
        }
        else if (config_result == CONFIG_PARSE_ERROR)
        {
            Serial.print("$result=error parse\n");
        }
    }

    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == create_fishbot_transport()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destory_fishbot_transport();
        };
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            if (!rmw_uros_epoch_synchronized())
            {
                RCSOFTCHECK(rmw_uros_sync_session(1000));
                if (rmw_uros_epoch_synchronized())
                {
                    setTime(rmw_uros_epoch_millis() / 1000 + SECS_PER_HOUR * 8);
                    fishlog_debug("fishbot", "current_time:%ld", rmw_uros_epoch_millis());
                }
                delay(10);
                return;
            }
            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        }
        break;
    case AGENT_DISCONNECTED:
        destory_fishbot_transport();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }

    if (state != AGENT_CONNECTED)
    {
        delay(10);
    }
}

bool microros_setup_transport_udp_client_()
{
    String ssid = config.wifi_sta_ssid();
    String ip = config.microros_uclient_server_ip();
    String password = config.wifi_sta_pswd();
    uint32_t agent_port = config.microros_uclient_server_port();

    // 初始化数据接收配置
    config_req.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_req.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);

    IPAddress agent_ip;
    agent_ip.fromString(ip);
    if (!set_microros_wifi_transports((char *)ssid.c_str(), (char *)password.c_str(), agent_ip, agent_port, config.board_name()))
    {
        return false;
    }
    return true;
}

bool microros_setup_transport_serial_()
{
    String nodename = config.ros2_nodename();
    String ros2namespace = config.ros2_namespace();
    String twist_topic = config.ros2_twist_topic_name();
    String odom_topic = config.ros2_odom_topic_name();
    String odom_frameid_str = config.ros2_odom_frameid();
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, odom_frameid_str.c_str());
    const unsigned int timer_timeout = config.odom_publish_period();
    // 初始化数据接收配置
    config_req.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_req.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    uint32_t serial_baudraate = config.serial_baudrate();
    Serial.updateBaudRate(serial_baudraate);
    if (!set_microros_serial_transports(Serial))
    {
        return false;
    }
    return true;
}

void callback_odom_publisher_timer_(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        odom_t odom = kinematics.odom();
        int64_t stamp = rmw_uros_epoch_millis();
        odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000); // 秒部分
        odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.position.y = odom.y;
        odom_msg.pose.pose.orientation.w = odom.quaternion.w;
        odom_msg.pose.pose.orientation.x = odom.quaternion.x;
        odom_msg.pose.pose.orientation.y = odom.quaternion.y;
        odom_msg.pose.pose.orientation.z = odom.quaternion.z;
        odom_msg.twist.twist.angular.z = odom.angular_speed;
        odom_msg.twist.twist.linear.x = odom.linear_x_speed;
        odom_msg.twist.twist.linear.y = odom.linear_y_speed;

        display.updateBotAngular(odom.angular_speed);
        display.updateBotLinear(odom.linear_x_speed);

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    }
}

void callback_twist_subscription_(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    static float target_motor_speed1, target_motor_speed2, target_motor_speed3, target_motor_speed4;
    kinematics.kinematic_inverse(msg->linear.x * 1000, msg->linear.y * 1000, msg->angular.z,
                                 target_motor_speed1, target_motor_speed2, target_motor_speed3, target_motor_speed4);

    pid_controller[0].update_target(target_motor_speed1);
    pid_controller[1].update_target(target_motor_speed2);
    pid_controller[2].update_target(target_motor_speed3);
    pid_controller[3].update_target(target_motor_speed4);
}

void callback_config_service_(const void *req, void *res)
{
    fishbot_interfaces__srv__FishBotConfig_Request *req_in = (fishbot_interfaces__srv__FishBotConfig_Request *)req;
    fishbot_interfaces__srv__FishBotConfig_Response *res_in = (fishbot_interfaces__srv__FishBotConfig_Response *)res;
    String recv_key(req_in->key.data);
    String recv_value(req_in->value.data);
    fishlog_debug("fishbot", "recv service key=%s,value=%s\n", req_in->key.data, req_in->value.data);
    config.config(recv_key, recv_value);
    // PID相关配置
    if (recv_key.startsWith("pid_"))
    {
        pid_controller[0].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
        pid_controller[1].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
        pid_controller[0].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
        pid_controller[1].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    }
    // 系统相关配置
    if (recv_key.equalsIgnoreCase("system"))
    {
        if (recv_value.equalsIgnoreCase("restart"))
        {
            xTaskCreate([](void *param)
                        {for(int i=0;i<3;i++)delay(1000);esp_restart(); },
                        "restart_task", 1024, NULL, 1, NULL);
        }
    }
    fishlog_debug("fishbot", "current_str:%s", config.config_str().c_str());
    sprintf(res_in->key.data, "%s", req_in->key.data);
    sprintf(res_in->value.data, "%s", req_in->key.data);
}