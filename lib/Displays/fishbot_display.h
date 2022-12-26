#ifndef __FISHBOT_DISPLAY_H__
#define __FISHBOT_DISPLAY_H__
#include <Wire.h>
#include <Arduino.h>
#include <SSD1306Wire.h>

class FishBotDisplay
{
private:
    SSD1306Wire* oled_;

    float battery_info_;
    float ultrasound_distance_;
    float bot_angular;
    float bot_linear_;

    uint64_t last_update_time;
    uint64_t update_interval{200}; // 5hz ,200ms更新一次
public:
    void init(SSD1306Wire* oled);
    void updateDisplay();
    void updateBatteryInfo(float &battery_info);
    void updateUltrasoundDist(float &ultrasound_distance);
    void botAngular(float &bot_angular);
    void botLinear(float &bot_linear);
    FishBotDisplay();
    ~FishBotDisplay() = default;
};

#endif // __FISHBOT_DISPLAY_H__