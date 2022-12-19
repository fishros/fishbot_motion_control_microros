#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

#include "Esp32PcntEncoder.h"
#include "Esp32McpwmMotor.h"
#include "PidController.h"
#include "Kinematics.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include "fishlog.h"
#include "oled/oled.h"
#include "fishbot_config.h"
#include "pio_rwm/serial/micro_ros_transport_serial.h"
#include "pio_rwm/wifi/micro_ros_transport_wifi_udp.h"


/*==================MicroROS消息============================*/
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;

/*==================MicroROS订阅发布者========================*/
rcl_publisher_t odom_publisher;
rcl_subscription_t twist_subscriber;

/*==================MicroROS相关执行器&节点===================*/
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

/*========================FishBot控制相关====================*/
Oled oled;
PidController pidController[2];
Esp32PcntEncoder encoders[2];
Esp32McpwmMotor motor;
Kinematics kinematics;
Preferences preferences;

#define RCSOFTCHECK(fn)                                                           \
  {                                                                               \
    rcl_ret_t temp_rc = fn;                                                       \
    if ((temp_rc != RCL_RET_OK))                                                  \
    {                                                                             \
      printf(                                                                     \
          "Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
    }                                                                             \
  }

void twist_subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  static float target_motor_speed1, target_motor_speed2;
  kinematics.kinematic_inverse(msg->linear.x * 1000, msg->angular.z, target_motor_speed1, target_motor_speed2);
  pidController[0].update_target(target_motor_speed1);
  pidController[1].update_target(target_motor_speed2);
  Serial.printf("recv cmd_vel(%f,%f) motor(%f,%f)\n", msg->linear.x * 1000, msg->angular.z, target_motor_speed1, target_motor_speed2);
}

void odom_publisher_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    odom_t odom = kinematics.odom();
    int64_t stamp = rmw_uros_epoch_millis();
    odom_msg.header.stamp.sec = stamp / 1000;
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

bool setup_microros_udp_client()
{

  String wifi_name = preferences.getString(CONFIG_NAME_WIFI_STA_SSID_NAME);
  String wifi_pswk = preferences.getString(CONFIG_NAME_WIFI_STA_PSWK_NAME);
  String wifi_server_ip = preferences.getString(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_IP);
  uint32_t microros_server_port = preferences.getULong(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_PORT);

  String odom_topic_name = preferences.getString(CONFIG_NAME_ROS2_ODOM_FRAMEID_NAME, CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
  String cmd_vel_topic_name = preferences.getString(CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME, CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME);

  IPAddress agent_ip;
  agent_ip.fromString(wifi_server_ip);
  set_microros_wifi_transports((char *)wifi_name.c_str(), (char *)wifi_pswk.c_str(), agent_ip, microros_server_port);

  delay(500);
  allocator = rcl_get_default_allocator();
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCSOFTCHECK(rclc_node_init_default(&node, CONFIG_DEFAULT_ROS2_NODE_NAME, CONFIG_DEFAULT_ROS2_NAMESPACE, &support));
  RCSOFTCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      odom_topic_name.c_str()));
  RCSOFTCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      cmd_vel_topic_name.c_str()));
  RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_TIMER_TIME), odom_publisher_timer_callback));
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, CONFIG_DEFAULT_ROS2_HANDLE_NUM, &allocator));
  return true;
}

void update_motor_speed_loop()
{
  static float out_motor_speed[2];
  kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks());
  out_motor_speed[0] = pidController[0].update(kinematics.motor_speed(0));
  out_motor_speed[1] = pidController[1].update(kinematics.motor_speed(1));
  motor.updateMotorSpeed(0, out_motor_speed[0]);
  motor.updateMotorSpeed(1, out_motor_speed[1]);
}

void microros_spin_task(void *param)
{
  while (true)
  {
    if (!rmw_uros_epoch_synchronized())
    {
      RCSOFTCHECK(rmw_uros_sync_session(CONFIG_DEFAULT_ROS2_TIME_SYNC_TIMEOUT));
      Serial.print("current_time:");
      Serial.println(rmw_uros_epoch_millis());
      continue;
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(CONFIG_DEFAULT_ROS2_RCL_SPIN_TIME)));
  }
}

/**
 * @brief 首次启动上电
 *
 */
void first_startup_setup()
{
  preferences.clear();
  Serial.printf("first startup,init config to default value.\n");
  // 设置传输模式，默认SERIAL模式
  preferences.putInt(CONFIG_NAME_TRANSPORT_MODE, CONFIG_DEFAULT_TRANSPORT_MODE);
  // 设置STA模式下名称
  preferences.putString(CONFIG_NAME_WIFI_STA_SSID_NAME, CONFIG_DEFAULT_WIFI_STA_SSID);
  // 设置STA模式下WIFI密码
  preferences.putString(CONFIG_NAME_WIFI_STA_PSWK_NAME, CONFIG_DEFAULT_WIFI_STA_PSWK);
  // 设置UDP模式下服务器IPIP
  preferences.putString(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_IP, CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP);
  // 设置UDP模式下服务器IP端口号
  preferences.putULong(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_PORT, CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT);
  // 设置里程计话题名称
  preferences.putString(CONFIG_NAME_ROS2_ODOM_FRAMEID_NAME, CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
  // 设置波特率
  preferences.putULong(CONFIG_NAME_TRANSPORT_SERIAL_BAUD, CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD);
  // 设置轮距
  preferences.putFloat(CONFIG_NAME_TRANSPORT_SERIAL_BAUD, CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE);
  // 设置轮距
  preferences.putFloat(CONFIG_NAME_KINEMATIC_WHEEL_DISTANCE, CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE);\
  // 设置Twist Topic
  preferences.putString(CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME, CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME);
  // 设置ODOM Topic
  preferences.putString(CONFIG_NAME_ROS2_ODOM_TOPIC_NAME, CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME);
  // 设置是否上次上电
  preferences.putBool(CONFIG_NAME_IS_FIRST_STARTUP, false);
}

void print_config()
{
  first_startup_setup();
  Serial.printf("config_name\t\tvalue\n-------------------------------------------------\n");
  Serial.printf("%s\t\t%d\n", CONFIG_NAME_IS_FIRST_STARTUP, preferences.getBool(CONFIG_NAME_IS_FIRST_STARTUP, false));
  Serial.printf("%s\t\t%d\n", CONFIG_NAME_TRANSPORT_MODE, preferences.getInt(CONFIG_NAME_TRANSPORT_MODE, CONFIG_DEFAULT_TRANSPORT_MODE));
  Serial.printf("%s\t\t%d\n", CONFIG_NAME_TRANSPORT_SERIAL_BAUD, preferences.getULong(CONFIG_NAME_TRANSPORT_SERIAL_BAUD, CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD));
  Serial.printf("%s\t\t%d\n", CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_PORT, preferences.getULong(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_PORT, CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_WIFI_STA_SSID_NAME, preferences.getString(CONFIG_NAME_WIFI_STA_SSID_NAME, CONFIG_DEFAULT_WIFI_STA_SSID));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_WIFI_STA_PSWK_NAME, preferences.getString(CONFIG_NAME_WIFI_STA_PSWK_NAME, CONFIG_DEFAULT_WIFI_STA_PSWK));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_IP, preferences.getString(CONFIG_NAME_TRANSPORT_MODE_WIFI_SERVER_IP, CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_ROS2_ODOM_FRAMEID_NAME, preferences.getString(CONFIG_NAME_ROS2_ODOM_FRAMEID_NAME, CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME, preferences.getString(CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME, CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME));
  Serial.printf("%s\t\t%s\n", CONFIG_NAME_ROS2_ODOM_TOPIC_NAME, preferences.getString(CONFIG_NAME_ROS2_ODOM_TOPIC_NAME, CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME));
  Serial.printf("%s\t\t%f\n", CONFIG_NAME_KINEMATIC_WHEEL_DISTANCE, preferences.getFloat(CONFIG_NAME_KINEMATIC_WHEEL_DISTANCE, CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE));
  Serial.printf("-------------------------------------------\n");
}

void setup()
{
  // 0.上电程序
  fishlog_debug("setup", "system syartup.");
  Serial.begin(CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD);
  Serial.println(FIRST_START_TIP);

  // 1.初始化配置相关设置
  preferences.begin(CONFIG_NAME_NAMESPACE);
  if (preferences.getBool(CONFIG_NAME_IS_FIRST_STARTUP, true))
  {
    first_startup_setup(); // 首次上电先初始化配置
  }
  else
  {
    print_config();
  }

  // 2.初始化oled
  oled.init(CONFIG_DEFAULT_OLED_SDA_GPIO, CONFIG_DEFAULT_OLED_SCL_GPIO);

  // 3.设置电机
  motor.attachMotors(CONFIG_DEFAULT_MOTOR0_A_GPIO, CONFIG_DEFAULT_MOTOR0_B_GPIO, CONFIG_DEFAULT_MOTOR1_A_GPIO, CONFIG_DEFAULT_MOTOR1_B_GPIO);

  // 4.设置编码器
  encoders[0].init(CONFIG_DEFAULT_ENCODER0_A_GPIO, CONFIG_DEFAULT_ENCODER0_B_GPIO, CONFIG_DEFAULT_PCNT_UTIL_00);
  encoders[1].init(CONFIG_DEFAULT_ENCODER1_A_GPIO, CONFIG_DEFAULT_ENCODER1_B_GPIO, CONFIG_DEFAULT_PCNT_UTIL_01);

  // 5.设置PID
  pidController[0].update_pid(CONFIG_DEFAULT_MOTOR_PID_KP, CONFIG_DEFAULT_MOTOR_PID_KI, CONFIG_DEFAULT_MOTOR_PID_KD);
  pidController[1].update_pid(CONFIG_DEFAULT_MOTOR_PID_KP, CONFIG_DEFAULT_MOTOR_PID_KI, CONFIG_DEFAULT_MOTOR_PID_KD);
  pidController[0].out_limit(CONFIG_DEFAULT_MOTOR_OUT_LIMIT_LOW, CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH);
  pidController[1].out_limit(CONFIG_DEFAULT_MOTOR_OUT_LIMIT_LOW, CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH);

  // 6.设置运动学参数
  kinematics.set_motor_param(0, CONFIG_DEFAULT_MOTOR0_PARAM_REDUCATION_RATIO, CONFIG_DEFAULT_MOTOR0_PARAM_PULSE_RATION, CONFIG_DEFAULT_MOTOR0_PARAM_WHEEL_DIAMETER);
  kinematics.set_motor_param(1, CONFIG_DEFAULT_MOTOR1_PARAM_REDUCATION_RATIO, CONFIG_DEFAULT_MOTOR1_PARAM_PULSE_RATION, CONFIG_DEFAULT_MOTOR1_PARAM_WHEEL_DIAMETER);
  kinematics.set_kinematic_param(CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE);

  // 7.设置microRos
  if (preferences.getInt(CONFIG_NAME_TRANSPORT_MODE) == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
  {
    setup_microros_udp_client();
  }
  else if (preferences.getInt(CONFIG_NAME_TRANSPORT_MODE) == CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT)
  {
    preferences.getULong(CONFIG_NAME_TRANSPORT_SERIAL_BAUD, CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD);
    set_microros_serial_transports(Serial);
  }

  // 8.添加/cmd_vel话题订阅
  RCSOFTCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_subscription_callback, ON_NEW_DATA));

  // 9.设置frameid,添加odom发布的定时器
  String odom_frame_id = preferences.getString(CONFIG_NAME_ROS2_CMD_VEL_TOPIC_NAME, CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID);
  odom_msg.header.frame_id.data = odom_frame_id.begin();
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));

  // 10.启动0号内核进行MicroRos相关处理
  xTaskCreatePinnedToCore(microros_spin_task, "microros_spin_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
  delay(10);
  update_motor_speed_loop();
}