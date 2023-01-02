#include "fishbot.h"

void fishbot_loop_transport_task(void *param)
{
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

Rate rate(100);
void loop()
{
  loop_fishbot_control();
  rate.sleep();
}
