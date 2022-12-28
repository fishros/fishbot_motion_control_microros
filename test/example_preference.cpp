#include "Arduino.h"
#include "Preferences.h"

void setup()
{
  Serial.begin(115200);
  Preferences p;

  p.begin("fishbot");
  String mode = p.getString("microros_mode", "");
  Serial.println(p.getString("microros_mode", ""));
  if (mode == "serial")
  {
    p.putString("microros_mode", "udp_client");
  }
  else if (mode == "udp_client")
  {
    p.putString("microros_mode", "serial");
  }
}

void loop()
{
  delay(10);
}
