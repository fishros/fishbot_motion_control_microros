#include "oled.h"

Oled::Oled()
{
}

void Oled::init(int sdaPin, int sclPin)
{
    Wire.begin(sdaPin, sclPin);
    _display = Adafruit_SSD1306(128, 64, &Wire);
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // 设置OLED的I2C地址
    _display.clearDisplay();                    // 清空屏幕
    _display.setTextSize(1);                    // 设置字体大小
    _display.setTextColor(SSD1306_WHITE);       // 设置字体颜色
    _display.setCursor(0, 0);                   // 设置开始显示文字的坐标
    _display.println("    fishbot-v1.0.0   ");  // 输出的字符
    _display.display();                         // 使更改的显示生效
}
void Oled::oledDrawText(int16_t x, int16_t y, const String &s)
{
    _display.setCursor(x, y); // 设置开始显示文字的坐标
    _display.println(s);      // 输出的字符
    _display.display();       // 使更改的显示生效
}
