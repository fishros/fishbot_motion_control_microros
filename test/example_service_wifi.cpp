#include <fishbot_interfaces/srv/fish_bot_config.h>
#include <fishbot_interfaces/srv/add_two_ints.h>
#include <stdio.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int64.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <micro_ros_transport_wifi_udp.h>
#include <micro_ros_utilities/type_utilities.h>

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

rcl_service_t service;
rcl_wait_set_t wait_set;

fishbot_interfaces__srv__FishBotConfig_Response res;
fishbot_interfaces__srv__FishBotConfig_Request req;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      while (1)                  \
      {                          \
      };                         \
    }                            \
  }
#define RCSOFTCHECK(fn)                                                                                                     \
  {                                                                                                                         \
    rcl_ret_t temp_rc = fn;                                                                                                 \
    if ((temp_rc != RCL_RET_OK))                                                                                            \
    {                                                                                                                       \
      printf("Failed status on line %d: %d. Message: %s, Aborting.\n", __LINE__, (int)temp_rc, rcl_get_error_string().str); \
    }                                                                                                                       \
  }

void service_callback(const void *req, void *res)
{
  fishbot_interfaces__srv__FishBotConfig_Request *req_in = (fishbot_interfaces__srv__FishBotConfig_Request *)req;
  fishbot_interfaces__srv__FishBotConfig_Response *res_in = (fishbot_interfaces__srv__FishBotConfig_Response *)res;

  Serial.printf("service_callback\n");
}
static micro_ros_utilities_memory_conf_t conf = {0};

void setup()
{
  // set_microros_transports();
  Serial.begin(115200);
  IPAddress agent_ip(192, 168, 2, 105);
  size_t agent_port = 8888;
  char ssid[] = "m3";
  char psk[] = "88888888";
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  delay(1000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "add_twoints_client_rclc", "", &support));

  // create service
  RCCHECK(rclc_service_init_default(&service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(fishbot_interfaces, srv, FishBotConfig), "/addtwoints"));

  static char memory[100];
  req.key.capacity = 100;
  req.key.data = memory;
  req.key.size = 0;
  static char memory2[100];
  req.value.capacity = 100;
  req.value.data = memory2;
  req.value.size = 0;
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_service(&executor, &service, &req, &res, service_callback));
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
