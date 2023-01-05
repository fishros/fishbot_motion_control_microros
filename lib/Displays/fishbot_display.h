/**
 * @file fishbot_display.h
 * @author fishros (fishros@foxmail.com)
 * @brief FishBot显示类定义
 * @version V1.0.0
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#ifndef __FISHBOT_DISPLAY_H__
#define __FISHBOT_DISPLAY_H__
#include <Wire.h>             // 加载Wire库
#include <Adafruit_GFX.h>     // 加载Adafruit_GFX库
#include <Adafruit_SSD1306.h> // 加载Adafruit_SSD1306库
#include <TimeLib.h>

class FishBotDisplay
{
private:
    Adafruit_SSD1306 _display;

    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    float bot_linear_;
    String mode_;
    String ip_="wait connect!";

    int64_t current_time ;
    uint64_t last_update_time;
    uint64_t update_interval{1000};
public:
    void init();
    void updateDisplay();
    void updateStartupInfo();
    void updateBatteryInfo(float &battery_info);
    void updateUltrasoundDist(float &ultrasound_distance);
    void updateBotAngular(float &bot_angular);
    void updateBotLinear(float &bot_linear);
    void updateTransMode(String mode);
    void updateCurrentTime(int64_t current_time_);
    void updateWIFIIp(String ip);
    String twoDigits(int digits);
    FishBotDisplay();
    ~FishBotDisplay() = default;
};

#endif // __FISHBOT_DISPLAY_H__