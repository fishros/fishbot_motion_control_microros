#ifndef __FISHBOT_DISPLAY_H__
#define __FISHBOT_DISPLAY_H__
#include <Wire.h>             // 加载Wire库
#include <Adafruit_GFX.h>     // 加载Adafruit_GFX库
#include <Adafruit_SSD1306.h> // 加载Adafruit_SSD1306库

class FishBotDisplay
{
private:
    Adafruit_SSD1306 _display;

    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    float bot_linear_;

    uint64_t last_update_time;
    uint64_t update_interval{1000}; // 5hz ,200ms更新一次
public:
    void init();
    void updateDisplay();
    void updateBatteryInfo(float &battery_info);
    void updateUltrasoundDist(float &ultrasound_distance);
    void updateBotAngular(float &bot_angular);
    void updateBotLinear(float &bot_linear);
    FishBotDisplay();
    ~FishBotDisplay() = default;
};

#endif // __FISHBOT_DISPLAY_H__