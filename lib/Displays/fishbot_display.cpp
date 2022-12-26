#include "fishbot_display.h"

void FishBotDisplay::init(SSD1306Wire *oled)
{
    oled_ = oled;
    oled_->init();
    oled_->clear();
    oled_->setFont(ArialMT_Plain_10);
    oled_->display();
    // _oled = Adafruit_SSD1306(128, 64, &Wire);
    // _oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); // 设置OLED的I2C地址
    // _oled.clearDisplay();                    // 清空屏幕
    // _oled.setTextSize(1);                    // 设置字体大小
    // _oled.setTextColor(SSD1306_WHITE);       // 设置字体颜色
    // _oled.setCursor(0, 0);                   // 设置开始显示文字的坐标
    // _oled.println("    fishbot-v1.0.0   ");  // 输出的字符
    // _oled.display();
}

FishBotDisplay::FishBotDisplay()
{
}

void FishBotDisplay::updateDisplay()
{
}
void FishBotDisplay::updateBatteryInfo(float &battery_info)
{
    battery_info_ = battery_info;
}
void FishBotDisplay::updateUltrasoundDist(float &ultrasound_distance)
{
    ultrasound_distance_ = ultrasound_distance;
}
void FishBotDisplay::botAngular(float &bot_angular)
{
    bot_linear_ = bot_angular;
}
void FishBotDisplay::botLinear(float &bot_linear)
{
    bot_linear_ = bot_linear;
}