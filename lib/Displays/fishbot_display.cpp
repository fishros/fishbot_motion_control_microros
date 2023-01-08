/**
 * @file fishbot_display.cpp
 * @author fishros (fishros@foxmail.com)
 * @brief FishBotOLED显示控制类
 * @version V1.0.0
 * @date 2023-01-05
 *
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 *
 */
#include "fishbot_display.h"

void FishBotDisplay::init()
{
    Wire.begin(18, 19, 400000UL);
    _display = Adafruit_SSD1306(128, 64, &Wire);
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // 设置OLED的I2C地址
    _display.clearDisplay();                    // 清空屏幕
    _display.setTextSize(1);                    // 设置字体大小
    _display.setTextColor(SSD1306_WHITE);       // 设置字体颜色
    _display.setCursor(0, 0);                   // 设置开始显示文字的坐标
    _display.println("  [fishbot-v1.0.0]");     // 输出的字符
    _display.println("");
    _display.println("...");
    _display.println("connect microros agent...");
    _display.display();
}

FishBotDisplay::FishBotDisplay()
{
}

void FishBotDisplay::updateDisplay()
{
    if (millis() - last_update_time > update_interval)
    {
        String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
        last_update_time = millis();
        _display.clearDisplay();
        _display.setCursor(0, 0);
        _display.println("   -fishbot-v1.0.0-");
        _display.print("microros:");
        _display.println(mode_);
        _display.print("time :");
        _display.println(timenow);
        _display.print("ip   :");
        _display.println(ip_);
        _display.print("voltage :");
        _display.println(battery_info_);
        _display.print("linear  :");
        _display.println(bot_linear_);
        _display.print("angular :");
        _display.println(bot_angular_);
        _display.display();
    }
}
void FishBotDisplay::updateBatteryInfo(float &battery_info)
{
    battery_info_ = battery_info;
}
void FishBotDisplay::updateUltrasoundDist(float &ultrasound_distance)
{
    ultrasound_distance_ = ultrasound_distance;
}
void FishBotDisplay::updateBotAngular(float &bot_angular)
{
    bot_angular_ = bot_angular;
}
void FishBotDisplay::updateBotLinear(float &bot_linear)
{
    bot_linear_ = bot_linear;
}
void FishBotDisplay::updateTransMode(String mode)
{
    mode_ = mode;
}
void FishBotDisplay::updateWIFIIp(String ip)
{
    if(ip!=ip_){
        ip_ = ip;

    }
}
void FishBotDisplay::updateCurrentTime(int64_t current_time_)
{
    current_time = current_time_;
}
String FishBotDisplay::twoDigits(int digits)
{
    if (digits < 10)
    {
        String i = '0' + String(digits);
        return i;
    }
    else
    {
        return String(digits);
    }
}

void FishBotDisplay::updateStartupInfo()
{
    String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    last_update_time = millis();
    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.println("   -fishbot-v1.0.0-");
    _display.print("microros:");
    _display.println(mode_);
    _display.print("voltage :");
    _display.println(battery_info_);
    _display.display();
}