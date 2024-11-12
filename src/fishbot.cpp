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
geometry_msgs__msg__Twist twist_msg; // 机器人的速度控制指令
nav_msgs__msg__Odometry odom_msg;    // 机器人的里程计信息
sensor_msgs__msg__Imu imu_msg;
char odom_frame_id[16];                                     // 用于存储机器人里程计信息中的坐标系名称
fishbot_interfaces__srv__FishBotConfig_Response config_res; // 机器人的配置信息
fishbot_interfaces__srv__FishBotConfig_Request config_req;  // 请求机器人的配置信息
static micro_ros_utilities_memory_conf_t conf = {0};        // 配置 Micro-ROS 库中的静态的内存管理器
static int keyvalue_capacity = 100;                         // 机器人配置信息中键值对（key-value）的最大容量，即最多能存储多少个键值对

/*==================MicroROS订阅发布者服务========================*/
rcl_publisher_t odom_publisher;      // 用于发布机器人的里程计信息（Odom）
rcl_publisher_t imu_publisher;       // 用于发布机器人的IMU信息（Imu）
rcl_subscription_t twist_subscriber; // 用于订阅机器人的速度控制指令（Twist）
rcl_service_t config_service;        // 用于提供机器人的配置信息
rcl_wait_set_t wait_set;             // 用于管理一组等待中的事件，例如发布者、订阅者、服务等

/*==================MicroROS相关执行器&节点===================*/
rclc_executor_t executor;  // 用于在单个线程中处理多个 ROS 2 资源的回调函数。
rclc_support_t support;    // 用于在 ROS 2 上下文中初始化和配置执行器、节点等资源
rcl_allocator_t allocator; // 用于在 ROS 2 节点中分配和释放内存
rcl_node_t node;           // 代表一个 ROS 2 系统中的节点，用于与其他节点通信
rcl_timer_t timer;         // 用于在指定的时间间隔内执行回调函数
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state; // 定义了一个枚举类型 states，用于表示机器人当前的状态

/*========================FishBot控制相关====================*/
// Stream *microros_serial;         // Serial 模式下microros使用的串口指针
PidController pid_controller[2]; // PID 控制器数组，用于控制机器人的左右两个电机。
Esp32PcntEncoder encoders[2];    // ESP32 PCNT 编码器数组，用于读取机器人的左右两个电机的旋转角度。
Esp32McpwmMotor motor;           // ESP32 MCPWM 电机对象，用于控制机器人的左右两个电机
Kinematics kinematics;           // 计算机器人的运动学模型
FishBotConfig config;            // 存储机器人的各种配置信息，例如电机功率、PID 参数等
FishBotDisplay display;          // 用于显示机器人的状态信息，例如电池电量、运动状态等
BluetoothSerial SerialBT;        // 蓝牙串口对象,用于通过蓝牙模块与其他设备进行通信
float battery_voltage;           // 存储机器人的电池电压
OneButton button(0, true);       // 单按钮（Button）对象，用于检测机器人上的一个按钮是否被按下
MPU6050 mpu(Wire);               // 初始化MPU6050对象
ImuDriver imu(mpu);              // 初始化Imu对象
imu_t imu_data;                  // IMU 数据对象

// WiFiEventCB 的回调函数,捕获 ESP32 系统中的 WiFi 事件
// WiFiEvent_t event 是一个 WiFi 事件类型的变量，用于存储捕获到的 WiFi 事件。
void WiFiEventCB(WiFiEvent_t event)
{
    Serial.println("WIFI EVENT!");
    // 根据不同的 WiFi 事件类型进行不同的处理
    switch (event)
    {
    // 处理当 ESP32 成功连接到 WiFi 并获取到 IP 地址时的情况
    case SYSTEM_EVENT_STA_GOT_IP:
        // 更新机器人的 IP 地址信息
        display.updateWIFIIp(WiFi.localIP().toString());
        break;
    // 处理当 ESP32 失去 WiFi 连接时的情况
    case SYSTEM_EVENT_STA_LOST_IP:
        // 调用 display.updateWIFIIp 函数，显示 "wait connect!"
        display.updateWIFIIp("wait connect!");
        break;
    };
}

// 用于处理机器人上的按钮双击事件
void doubleClick()
{
    fishlog_debug("key", "doubleClick() detected.");
    // 用于判断当前 Micro-ROS 传输模式是否为 WiFi UDP 客户端模式
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
    {
        // 当前 Micro-ROS 传输模式为 WiFi UDP 客户端模式，函数将会将传输模式改为串口模式
        config.config("microros_mode", "serial");
    }
    else
    {
        // 函数将会将传输模式改为 WiFi UDP 客户端模式
        config.config("microros_mode", "udp_client");
    }
    // 用于重新启动 ESP32 系统。
    esp_restart();
}

void oneClick()
{
    static uint8_t display_mode = 0;
    display.updateDisplayMode(display_mode++);
}

bool setup_fishbot()
{
    // 1.初始化
    Serial.begin(115200);
    fishlog_set_target(Serial);
    config.init(CONFIG_NAME_NAMESPACE);
    Serial.println(FIRST_START_TIP);
    Serial.println(config.config_str());
    if (config.microros_serial_id() == 2)
    {
        Serial2.begin(115200);
    }
    // 初始化LED
    pinMode(2,OUTPUT);
    // 初始化显示
    display.updateVersionCode(VERSION_CODE);
    display.init();
    display.updateTransMode(config.microros_transport_mode());
    display.updateBaudRate(config.serial_baudrate());
    display.updateStartupInfo();
    // 初始化按键
    button.attachDoubleClick(doubleClick);
    button.attachClick(oneClick);
    // 初始化IMU
    imu.begin(18, 19);
    // 2.设置IO 电机&编码器
    motor.attachMotor(0, CONFIG_DEFAULT_MOTOR0_A_GPIO, CONFIG_DEFAULT_MOTOR0_B_GPIO);
    motor.attachMotor(1, CONFIG_DEFAULT_MOTOR1_A_GPIO, CONFIG_DEFAULT_MOTOR1_B_GPIO);
    encoders[0].init(CONFIG_DEFAULT_PCNT_UTIL_00, CONFIG_DEFAULT_ENCODER0_A_GPIO, CONFIG_DEFAULT_ENCODER0_B_GPIO);
    encoders[1].init(CONFIG_DEFAULT_PCNT_UTIL_01, CONFIG_DEFAULT_ENCODER1_A_GPIO, CONFIG_DEFAULT_ENCODER1_B_GPIO);
    // 3.设置PID
    pid_controller[0].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
    pid_controller[1].update_pid(config.kinematics_pid_kp(), config.kinematics_pid_ki(), config.kinematics_pid_kd());
    pid_controller[0].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    pid_controller[1].out_limit(-config.kinematics_pid_out_limit(), config.kinematics_pid_out_limit());
    // 4.设置运动学参数
    kinematics.set_motor_param(0, config.kinematics_reducation_ration(), config.kinematics_pulse_ration(), config.kinematics_wheel_diameter());
    kinematics.set_motor_param(1, config.kinematics_reducation_ration(), config.kinematics_pulse_ration(), config.kinematics_wheel_diameter());
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
    // 检查传入的 "key" 是否为 "command"
    if (strcmp(key, "command") == 0)
    {
        // 检查 "value" 是否为 "restart"
        if (strcmp(value, "restart") == 0)
        {
            // 通过调用 "esp_restart" 函数来重新启动 ESP8266
            esp_restart();
        }
        // 检查 "value" 是否 "read_config"
        else if (strcmp(value, "read_config") == 0)
        {
            // 调用 "config.config_str()" 函数来获取当前配置，并通过串口输出到终端
            // "config.config_str()" 函数的作用是将当前配置转化为一个字符串类型，以便于输出或存储
            Serial.print(config.config_str());
        }
        return;
    }
    // 传入的命令不是 "restart" 或 "read_config"
    else
    {
        // 创建两个 String 对象 "recv_key" 和 "recv_value"
        // 并将传入的 "key" 和 "value" 分别作为参数来初始化这两个对象。
        String recv_key(key);
        String recv_value(value);
        // 调用 "config.config" 方法，将 "recv_key" 和 "recv_value" 作为参数传递给它，将它们存储到当前配置中
        config.config(recv_key, recv_value);
        Serial.print("$result=ok\n");
    }
}

// 用于设置小鱼机器人传输模式，并返回设置是否成功的状态
bool setup_fishbot_transport()
{
    bool setup_success = true;
    // 函数检查配置中的传输模式是否为 "CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT"
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
    {
        // 将日志输出目标设置为串口，并注册一个回调函数 "WiFiEventCB"，以便在 WiFi 连接状态发生变化时进行处理。
        fishlog_set_target(Serial);
        WiFi.onEvent(WiFiEventCB);
        // 启动 UDP 客户端传输模式，并更新显示器的传输模式为 "udp_client"
        setup_success = microros_setup_transport_udp_client_();
        display.updateTransMode("udp_client");
    }
    // 检查配置中的传输模式是否为 "CONFIG_TRANSPORT_MODE_SERIAL
    if (config.microros_transport_mode() == CONFIG_TRANSPORT_MODE_SERIAL)
    {
        // 使用 "config.board_name()" 函数获取板子名称,使用 "SerialBT.begin()" 启动蓝牙传输模式
        SerialBT.begin(config.board_name());
        // 将日志输出目标设置为蓝牙串口
        fishlog_set_target(SerialBT);
        if (config.microros_serial_id() == 2)
        {
            // 调用 "microros_setup_transport_serial_()" 函数，以启动串口传输模式
            microros_setup_transport_serial_(Serial2);
            display.updateTransMode("serial2");
        }
        else
        {
            microros_setup_transport_serial_(Serial);
            display.updateTransMode("serial");
        }
    }
    // 返回 true，表示传输模式设置成功
    return true;
}

// 用于创建 FishBot 的通信节点和相关通信组件的函数。
bool create_fishbot_transport()
{
    // 获取配置文件中定义的 ROS 2 节点名称、命名空间、扭矩和里程计主题名称、里程计帧 ID 等信息
    String nodename = config.ros2_nodename();
    String ros2namespace = config.ros2_namespace();
    String twist_topic = config.ros2_twist_topic_name();
    String odom_topic = config.ros2_odom_topic_name();
    String odom_frameid_str = config.ros2_odom_frameid(); // 存储里程计帧 ID
    String odom_child_frameid_str = config.ros2_odom_child_frameid();
    // 使用 micro_ros_string_utilities_set 函数设置到 odom_msg.header.frame_id 中
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, odom_frameid_str.c_str());
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, odom_child_frameid_str.c_str());
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu");
    const unsigned int timer_timeout = config.odom_publish_period();
    delay(500);
    // 默认的内存分配器 allocator
    allocator = rcl_get_default_allocator();
    // RCSOFTCHECK 是一个宏定义，用于检查执行函数的返回值是否出错，如果出错，则会打印错误信息并退出程序。
    // 调用 rclc_support_init 函数初始化 ROS 2 运行时的支持库，传入 allocator
    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // 调用 rclc_node_init_default 函数初始化 ROS 2 节点，传入节点名称、命名空间和支持库
    RCSOFTCHECK(rclc_node_init_default(&node, nodename.c_str(), ros2namespace.c_str(), &support));
    // 调用 rclc_publisher_init_best_effort 函数初始化 ROS 2 发布者，传入节点、消息类型和主题名称。
    RCSOFTCHECK(rclc_publisher_init_best_effort(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        odom_topic.c_str()));
    RCSOFTCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"));
    // 调用 rclc_subscription_init_best_effort 函数初始化 ROS 2 订阅者，传入节点、消息类型和主题名称。
    RCSOFTCHECK(rclc_subscription_init_best_effort(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        twist_topic.c_str()));
    // 调用 rclc_service_init_default 函数初始化 ROS 2 服务，传入节点、服务类型和服务名称
    RCSOFTCHECK(rclc_service_init_default(
        &config_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(fishbot_interfaces, srv, FishBotConfig),
        "/fishbot_config"));
    // 调用 rclc_timer_init_default 函数初始化 ROS 2 定时器，传入支持库、定时器周期和回调函数
    RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), callback_sensor_publisher_timer_));
    // 调用 rclc_executor_init 函数初始化 ROS 2 执行器，传入支持库、执行器线程数和内存分配器
    RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    // 调用 rclc_executor_add_subscription 函数将订阅者添加到执行器中，传入执行器、订阅者、消息和回调函数。
    RCSOFTCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &callback_twist_subscription_, ON_NEW_DATA));
    // 调用 rclc_executor_add_timer 函数将定时器添加到执行器中，传入执行器和定时器。
    RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
    // 调用 rclc_executor_add_service 函数添加一个服务（Service）的操作
    RCSOFTCHECK(rclc_executor_add_service(&executor, &config_service, &config_req, &config_res, callback_config_service_));
    return true;
}

// 用于销毁 ROS 2 节点中的一些资源
bool destory_fishbot_transport()
{
    // 获取 ROS 2 上下文中的 RMW 上下文，并将其赋值给 rmw_context 变量。
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    // 设置 ROS 2 上下文中的 RMW 上下文的实体销毁会话超时时间为 0，这意味着实体销毁操作将立即返回，而不是等待超时
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    // RCSOFTCHECK 是一个宏定义，用于检查执行函数的返回值是否出错，如果出错，则会打印错误信息并退出程序。
    // 用于销毁一个 ROS 2 发布者（Publisher）
    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    // 用于销毁一个 ROS 2 订阅者（Subscriber）
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    // 用于销毁一个 ROS 2 服务（Service）
    RCSOFTCHECK(rcl_service_fini(&config_service, &node));
    // 用于销毁一个 ROS 2 定时器（Timer）
    RCSOFTCHECK(rcl_timer_fini(&timer));
    // 用于停止执行器（Executor）并释放相关资源
    RCSOFTCHECK(rclc_executor_fini(&executor));
    // 用于销毁一个 ROS 2 节点（Node）
    RCSOFTCHECK(rcl_node_fini(&node));
    // 用于释放支持库中分配的资源
    rclc_support_fini(&support);
    return true;
}

void loop_fishbot_control()
{
    // 用于存储电机速度输出和上次更新信息的时间戳
    static float out_motor_speed[2];
    static uint64_t last_update_info_time = millis();
    static uint8_t index;
    // 新机器人运动学模型中电机的转速和编码器的读数
    kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks());
    // 将输出值存储在 out_motor_speed[0] 和 out_motor_speed[1] 变量中
    for (index = 0; index < 2; index++)
    {
        // 目标速度为0时停止控制，解决 #https://fishros.org.cn/forum/topic/1372 问题
        if (pid_controller[index].target_ == 0)
        {
            out_motor_speed[index] = 0;
        }
        else
        {
            // 使用 pid_controller[0] 和 pid_controller[1] 控制器对电机速度进行 PID 控制
            out_motor_speed[index] = pid_controller[index].update(kinematics.motor_speed(index));
        }
        // 将 PID 控制器的输出值作为电机的目标速度进行控制
        motor.updateMotorSpeed(index, out_motor_speed[index]);
    }
    // 电量信息
    // 电机速度为零，则读取电池电压
    if (out_motor_speed[0] == 0 && out_motor_speed[1] == 0)
    {
        // 将电池电压信息显示在屏幕上
        battery_voltage = 5.02 * ((float)analogReadMilliVolts(34) * 1e-3);
        display.updateBatteryInfo(battery_voltage);
    }
    // 更新系统信息
    display.updateCurrentTime(rmw_uros_epoch_millis());
    // 刷新显示屏幕
    display.updateDisplay();
    // 用于处理按钮事件等操作
    button.tick();
    imu.update();
}

void loop_fishbot_transport()
{
    static char result[10][32];
    static int config_result; // 用于存储解析配置数据的结果
    // 不断读取串口数据，直到串口中没有数据可读
    while (Serial.available())
    {
        int c = Serial.read();
        // 调用config对象的loop_config_uart函数，该函数用于解析读取到的配置数据，并将解析结果存储在result数组中
        // config_result表示解析结果的状态
        config_result = config.loop_config_uart(c, result);
        if (config_result == CONFIG_PARSE_OK)
        {
            // 调用deal_command函数处理解析结果。
            deal_command(result[0], result[1]);
        }
        // 调用deal_command函数处理解析结果。
        else if (config_result == CONFIG_PARSE_ERROR)
        {
            Serial.print("$result=error parse\n");
        }
    }
    // 函数根据当前的状态执行不同的操作
    switch (state)
    {

    // 对于WAITING_AGENT状态，函数会每500毫秒执行一次RMW_RET_OK == rmw_uros_ping_agent(100, 1)语句
    // 该语句用于向MicroROS代理发送ping消息，并检查是否能够收到pong消息。
    // 如果收到pong消息，则将状态设置为AGENT_AVAILABLE；否则保持等待状态。
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        // fishlog_debug("ros2", "current state2:%d", state);
        break;

    // 对于AGENT_AVAILABLE状态，函数将尝试创建fishbot传输，并将状态设置为AGENT_CONNECTED。
    // 如果创建成功，则继续保持AGENT_CONNECTED状态；否则将状态设置为WAITING_AGENT，并销毁fishbot传输。
    case AGENT_AVAILABLE:
        state = (true == create_fishbot_transport()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destory_fishbot_transport();
        };
        break;
    // 对于AGENT_CONNECTED状态，函数会每200毫秒执行一次RMW_RET_OK == rmw_uros_ping_agent(100, 1)语句
    // 该语句用于向MicroROS代理发送ping消息，并检查是否能够收到pong消息。
    // 如果收到pong消息，则保持AGENT_CONNECTED状态，并尝试同步时间。
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(2000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 5)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            if (!rmw_uros_epoch_synchronized())
            {
                RCSOFTCHECK(rmw_uros_sync_session(1000));
                // 如果时间同步成功，则将当前时间设置为MicroROS代理的时间，并输出调试信息。
                if (rmw_uros_epoch_synchronized())
                {
                    // 该函数的参数是一个Unix时间戳，表示自1970年1月1日0时0分0秒以来的秒数
                    // rmw_uros_epoch_millis()返回MicroROS代理的当前时间（毫秒），除以1000将其转换为秒数。
                    // SECS_PER_HOUR表示一小时的秒数，乘以8表示东八区的时差。
                    // 因此，整个计算表达式的结果是将MicroROS代理的时间转换为本地时间
                    setTime(rmw_uros_epoch_millis() / 1000 + SECS_PER_HOUR * 8);
                    fishlog_debug("fishbot", "current_time:%ld", rmw_uros_epoch_millis());
                }
                delay(10);
                return;
            }
            // 闪烁LED
            digitalWrite(2,!digitalRead(2)); 
            // 函数调用rclc_executor_spin_some函数，在100毫秒内执行一些待处理的 ROS2 消息。
            RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        }
        break;
    // 当状态为AGENT_DISCONNECTED时，函数将销毁fishbot传输并将状态设置为WAITING_AGENT，表示等待MicroROS代理连接。
    // AGENT_DISCONNECTED状态下无法与MicroROS代理通信，需要重新连接
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
// 用于设置MicroROS客户端的UDP传输。它不接受任何参数
bool microros_setup_transport_udp_client_()
{
    // 从config对象中获取WiFi的SSID、密码、MicroROS客户端连接的IP地址和端口号等配置信息，并初始化数据接收配置
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
    // 将IP地址转换为IPAddress类型
    agent_ip.fromString(ip);
    // 调用set_microros_wifi_transports函数，该函数用于设置MicroROS的WiFi传输，并将WiFi的SSID、密码、代理IP地址、代理端口和设备名称传递给它
    if (!set_microros_wifi_transports((char *)ssid.c_str(), (char *)password.c_str(), agent_ip, agent_port, config.board_name()))
    {
        return false;
    }
    return true;
}

bool microros_setup_transport_serial_(HardwareSerial &serial)
{
    // 从config对象中获取ROS 2节点名称、ROS 2命名空间、Twist和Odom等主题名称以及Odom帧ID等配置信息。
    String nodename = config.ros2_nodename();
    String ros2namespace = config.ros2_namespace();
    String twist_topic = config.ros2_twist_topic_name();
    String odom_topic = config.ros2_odom_topic_name();
    String odom_frameid_str = config.ros2_odom_frameid();
    String odom_child_frameid_str = config.ros2_odom_child_frameid();
    // 用于将odom_frameid_str的C字符串值设置为odom_msg的帧ID。
    // 该函数会将odom_msg中的帧ID指针所指向的字符串缓冲区中的数据替换为odom_frameid_str的C字符串值，并返回指向新字符串的指针。
    // 这样可以确保odom_msg的帧ID与配置中的值一致。
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, odom_frameid_str.c_str());
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, odom_frameid_str.c_str());
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu");
    const unsigned int timer_timeout = config.odom_publish_period();
    // 初始化数据接收配置
    // 初始化了四个名为config_req和config_res的结构体变量。
    // 这些结构体变量包含两个名为key和value的指针，它们用于在MicroROS配置请求和响应中存储键值对数据
    // micro_ros_string_utilities_init_with_size函数用于初始化一个指定大小的字符串缓冲区，并返回指向该缓冲区的指针。
    // 在这里，keyvalue_capacity是一个预定义的常量，它表示缓冲区的大小。
    // 这些缓冲区将用于存储MicroROS配置请求和响应中的键值对数据
    config_req.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_req.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.key = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    config_res.value = micro_ros_string_utilities_init_with_size(keyvalue_capacity);
    // 获取串口波特率
    uint32_t serial_baudraate = config.serial_baudrate();
    // 使用Serial.updateBaudRate函数更新串口波特率
    serial.updateBaudRate(serial_baudraate);
    if (!set_microros_serial_transports(serial))
    {
        return false;
    }
    return true;
}

// 用于在定时器触发时发布机器人的位置和速度信息
void callback_sensor_publisher_timer_(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // 用于获取当前的时间戳，并将其存储在消息的头部中
        int64_t stamp = rmw_uros_epoch_millis();
        // 获取机器人的位置和速度信息，并将其存储在一个ROS消息（odom_msg）中
        odom_t odom = kinematics.odom();
        odom_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);              // 秒部分
        odom_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分
        odom_msg.pose.pose.position.x = odom.x;
        odom_msg.pose.pose.position.y = odom.y;
        odom_msg.pose.pose.orientation.w = odom.quaternion.w;
        odom_msg.pose.pose.orientation.x = odom.quaternion.x;
        odom_msg.pose.pose.orientation.y = odom.quaternion.y;
        odom_msg.pose.pose.orientation.z = odom.quaternion.z;

        odom_msg.twist.twist.angular.z = odom.angular_speed;
        odom_msg.twist.twist.linear.x = odom.linear_speed;

        display.updateBotAngular(odom.angular_speed);
        display.updateBotLinear(odom.linear_speed);

        RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

        if (imu.isEnable())
        {
            imu.getImuDriverData(imu_data);

            imu_msg.header.stamp.sec = static_cast<int32_t>(stamp / 1000);              // 秒部分
            imu_msg.header.stamp.nanosec = static_cast<uint32_t>((stamp % 1000) * 1e6); // 纳秒部分

            imu_msg.angular_velocity.x = imu_data.angular_velocity.x;
            imu_msg.angular_velocity.y = imu_data.angular_velocity.y;
            imu_msg.angular_velocity.z = imu_data.angular_velocity.z;

            imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x;
            imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y;
            imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z;

            imu_msg.orientation.x = imu_data.orientation.x;
            imu_msg.orientation.y = imu_data.orientation.y;
            imu_msg.orientation.z = imu_data.orientation.z;
            imu_msg.orientation.w = imu_data.orientation.w;

            RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
        }
    }
}

// 用于处理接收到的Twist类型的ROS消息
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