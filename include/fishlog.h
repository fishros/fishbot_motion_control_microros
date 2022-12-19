#include "Arduino.h"

#define fishlog_info(log_name, fmt, arg...) \
    do                                      \
    {                                       \
        Serial.printf(                      \
            "[%d]>I:" fmt, millis(),        \
            ##arg);                         \
    } while (0);

#define fishlog_debug(log_name, fmt, arg...) \
    do                                       \
    {                                        \
        Serial.printf(                       \
            "[%d]>D:" fmt, millis(),         \
            ##arg);                          \
    } while (0);
