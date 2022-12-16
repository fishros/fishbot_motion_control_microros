#ifndef __CONFIG_H__
#define __CONFIG_H_
#include "LittleFS.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    int is_config; // 是否配置，0未配置，1已配置
    char ssid[32];
    char pswd[64];
    int proto_mode;     // 通讯模式 serial(0),wifi(1)两种
    int bautrate;       // serial模式下波特率
    char server_ip[20]; // wifi模式下服务ip和port
    int server_port;    
} fishbot_config_t;


class FishBotConfig
{
public:
    FishBotConfig() = default;

public:
    bool initConfig();
    bool readConfig();
    bool parseCommand(uint8_t count, char result[][32]);
private:
    bool _commandRestart();
    bool _commandConfigWifi(String ssid, String pswd);
    bool _commandConfigModeWifi(String serverIp,String serverPort);
    bool _commandConfigModeSerial(String bautrate);
    bool _saveConfig(fishbot_config_t *config);

private:
    fishbot_config_t config_;
};

#endif