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

enum fishbot_wifi_status_t
{
    FISHBOT_WIFI_STATUS_OK,
    FISHBOT_WIFI_STATUS_NO_FOUND,
    FISHBOT_WIFI_STATUS_PASD_ERROR,
    FISHBOT_WIFI_STATUS_WAIT_CONNECT,
    FISHBOT_WIFI_STATUS_PING_FAILED,
    FISHBOT_WIFI_STATUS_UNKNOW,
};

class FishBotDisplay
{
private:
    Adafruit_SSD1306 _display;

    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    float bot_linear_;
    uint32_t baudrate_;
    String mode_;
    String version_code_;
    uint8_t display_mode_;

    int64_t current_time;
    uint64_t last_update_time;
    uint64_t update_interval{1000};

    String wifi_ssid_;
    String wifi_pswd_;
    String wifi_ip_;
    String wifi_server_ip_;
    String wifi_info_ = "wait connect";
    fishbot_wifi_status_t wifi_status_ = FISHBOT_WIFI_STATUS_WAIT_CONNECT;

public:
    void init();
    void updateDisplayMode(uint8_t display_mode);
    void updateDisplay();
    void updateStartupInfo();
    void updateBatteryInfo(float &battery_info);
    void updateUltrasoundDist(float &ultrasound_distance);
    void updateBotAngular(float &bot_angular);
    void updateBotLinear(float &bot_linear);
    void updateTransMode(String mode);
    void updateCurrentTime(int64_t current_time_);
    void updateBaudRate(uint32_t baudrate);
    void updateWIFIIp(String ip);
    void updateWIFIServerIp(String server_ip);
    void updateWIFIInfo(String info, fishbot_wifi_status_t status);
    void updateWIFISSID(String ssid);
    void updateWIFIPSWD(String pswd);
    void updateVersionCode(String version_code);
    String twoDigits(int digits);
    FishBotDisplay();
    ~FishBotDisplay() = default;
};

#endif // __FISHBOT_DISPLAY_H__