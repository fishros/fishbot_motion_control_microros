/**
 * @file main.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief 主函数部分
 * @version V1.0.0
 * @date 2023-01-04
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#include <Arduino.h>
#include "fishbot.h"

void fishbot_loop_transport_task(void *param)
{
  setup_fishbot_transport();
  while (true)
  {
    loop_fishbot_transport();
  }
}

void setup()
{
  setup_fishbot();
  xTaskCreatePinnedToCore(fishbot_loop_transport_task, "fishbot_loop_transport_task", 10240, NULL, 1, NULL, 0);
}

void loop()
{
  delay(10);
  loop_fishbot_control();
}
