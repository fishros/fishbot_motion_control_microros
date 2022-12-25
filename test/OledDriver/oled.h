#ifndef __OLED_H__
#define __OLED_H__

#include <Wire.h>             // 加载Wire库
#include <Adafruit_GFX.h>     // 加载Adafruit_GFX库
#include <Adafruit_SSD1306.h> // 加载Adafruit_SSD1306库

class Oled
{
public:
    Oled();
    void init(int sdaPin, int sclPin);
    void oledDrawText(int16_t x, int16_t y, const String &s);

private:
    Adafruit_SSD1306 _display;
};

#endif