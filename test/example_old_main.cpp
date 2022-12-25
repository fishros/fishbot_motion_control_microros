#include <Arduino.h>
#include <WiFi.h>

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
#include "micro_ros_transport_serial.h"
#include "micro_ros_transport_wifi_udp.h"

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
PidController pidController[2];
Esp32PcntEncoder encoders[2];
Esp32McpwmMotor motor;
Kinematics kinematics;

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

bool setup_microros_wifi(String ssid, String password, String ip)
{
  IPAddress agent_ip;
  agent_ip.fromString(ip);
  size_t agent_port = 8888;
  set_microros_wifi_transports((char *)ssid.c_str(), (char *)password.c_str(), agent_ip, agent_port);
  delay(500);
  allocator = rcl_get_default_allocator();
  RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCSOFTCHECK(rclc_node_init_default(&node, "fishbot_motion_control", "", &support));
  RCSOFTCHECK(rclc_publisher_init_default(
      &odom_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "odom"));
  RCSOFTCHECK(rclc_subscription_init_default(
      &twist_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"));
  const unsigned int timer_timeout = 50;
  RCSOFTCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), odom_publisher_timer_callback));
  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
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
      RCSOFTCHECK(rmw_uros_sync_session(1000));
      Serial.print("current_time:");
      Serial.println(rmw_uros_epoch_millis());
      continue;
    }
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}

void setup()
{
  // 1.初始化串口
  Serial.begin(115200);
  Serial.printf("=================================================\n");
  Serial.printf("     wwww.fishros.com        \n");
  Serial.printf("fishbot-motion-control-v1.0.0\n");
  Serial.printf("=================================================\n");
  // 2.初始化oled
  // 3.设置电机
  motor.attachMotors(22, 23, 12, 13);
  // 4.设置编码器
  encoders[0].init(32, 33, 0);
  encoders[1].init(26, 25, 1);
  // 5.设置PID
  pidController[0].update_pid(0.625, 0.125, 0.0);
  pidController[1].update_pid(0.625, 0.125, 0.0);
  pidController[0].out_limit(-100, 100);
  pidController[1].out_limit(-100, 100);
  // 6.设置运动学参数
  kinematics.set_motor_param(0, 46, 42, 65);
  kinematics.set_motor_param(1, 46, 42, 65);
  kinematics.set_kinematic_param(150);
  // 7.设置microRos
  setup_microros_wifi("m3", "88888888", "192.168.2.105");
  // 8.添加/cmd_vel话题订阅
  RCSOFTCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_subscription_callback, ON_NEW_DATA));
  // 9.设置frameid,添加odom发布的定时器
  odom_msg.header.frame_id.data = "odom";
  RCSOFTCHECK(rclc_executor_add_timer(&executor, &timer));
  // 10.启动0号内核进行MicroRos相关处理
  xTaskCreatePinnedToCore(microros_spin_task, "microros_spin_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
  delay(10);
  update_motor_speed_loop();
}