/**
 * @file fishlog.h
 * @author fishros (fishros@foxmail.com)
 * @brief 日志管理类
 * @version V1.0.0
 * @date 2023-01-04
 * 
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 * 
 */
#ifndef __FISHLOG_H__
#define __FISHLOG_H__

#include "Arduino.h"

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO 1
#define LOG_LEVEL_WARN 2
#define LOG_LEVEL_NONE 8

class FishLog
{
private:
    char log_buff[512];
    Stream *target{nullptr};
    FishLog() = default;

public:
    static FishLog &getInstance()
    {
        static FishLog instance;
        return instance;
    }
    void setLogTarget(Stream &stream)
    {
        target = &stream;
    };
    ~FishLog() = default;
    FishLog(const FishLog &) = delete;
    FishLog &operator=(const FishLog &) = delete;

public:
    int log(uint8_t level, const char *format, ...)
    {
        if (target != nullptr)
        {
            char loc_buf[64];
            char *temp = loc_buf;
            va_list arg;
            va_list copy;
            va_start(arg, format);
            va_copy(copy, arg);
            int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
            va_end(copy);
            if (len < 0)
            {
                va_end(arg);
                return 0;
            };
            if (len >= sizeof(loc_buf))
            {
                temp = (char *)malloc(len + 1);
                if (temp == NULL)
                {
                    va_end(arg);
                    return 0;
                }
                len = vsnprintf(temp, len + 1, format, arg);
            }
            va_end(arg);
            len = target->write((uint8_t *)temp, len);
            if (temp != loc_buf)
            {
                free(temp);
            }
            return len;
        }
        return -1;
    }
};

#define fishlog_set_target(stream) FishLog::getInstance().setLogTarget(stream);

#define fishlog_debug(log_name, fmt, arg...)                                                           \
    do                                                                                                 \
    {                                                                                                  \
        FishLog::getInstance().log(LOG_LEVEL_DEBUG, "[%d]@%d>D:" fmt "\n", millis(), __LINE__, ##arg); \
    } while (0);

#endif // __FISHLOG_H__