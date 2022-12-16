#include "config.h"
#define CONFIG_FILE_NAME "/config.txt"


static uint8_t splitStr(char *line, char result[][32])
{
    uint16_t index = 0;
    uint16_t count = 0;
    uint16_t temp_index = 0;
    for (index = 0; line[index] != '\0'; index++)
    {
        if (line[index] == ',')
        {
            result[count++][temp_index++] = '\0';
            temp_index = 0;
            continue;
        }
        result[count][temp_index++] = line[index];
    }
    result[count][temp_index++] = '\0';
    return count + 1;
}

bool FishBotConfig::parseCommand(uint8_t count, char result[][32])
{
    if (strcmp(result[0], "restart") == 0)
    {
        _commandRestart();
    }

    if (strcmp(result[0], "config") == 0)
    {
        if (strcmp(result[1], "wifi") == 0)
        {
            _commandConfigWifi(result[2], result[3]);
        }
        if (strcmp(result[1], "proto") == 0)
        {
            if (strcmp(result[2], "wifi") == 0)
            {
                _commandConfigModeWifi(result[3], result[4]);
            }
        }
    }
    return true;
}

bool FishBotConfig::initConfig()
{
    if (readConfig() == false)
    {
        _commandConfigWifi("fishbot", "12345678");
        _commandConfigModeSerial("115200");
        return false;
    }
    return true;
}

bool FishBotConfig::readConfig()
{
    LittleFS.begin();
    File file = LittleFS.open(CONFIG_FILE_NAME, "r");
    if (!file)
    {
        Serial.println("config file open failed");
        return false;
    }
    file.readBytes((char *)&config_, sizeof(fishbot_config_t));
    Serial.printf("ssid,%s,wifi name.\n", config_.ssid);
    Serial.printf("pswd,%s,wifi password.\n", config_.pswd);
    Serial.printf("proto_mode,%s,transport protocol mode.\n", config_.proto_mode);
    Serial.printf("server_ip,%s,wifi mode server ip address.\n", config_.server_ip);
    Serial.printf("server_port,%s,wifi mode server ip address.\n", config_.server_port);
    Serial.printf("bautrate,%s,serial mode bautrate.\n", config_.bautrate);
    if (config_.is_config == 1)
    {
        Serial.println("alread config!");
        return true;
    }
    file.close();
    LittleFS.end();
    Serial.println("not config!");
    return false;
}

bool FishBotConfig::_saveConfig(fishbot_config_t *config)
{
    LittleFS.begin();
    File file = LittleFS.open(CONFIG_FILE_NAME, "w");
    if (!file)
    {
        Serial.println("file open failed");
    }
    file.write((uint8_t *)config, sizeof(fishbot_config_t));
    file.close();
    LittleFS.end();
}

bool FishBotConfig::_commandRestart()
{
    Serial.printf("三秒后将进行重启...");
    delay(3000);
    esp_restart();
    return true;
}

bool FishBotConfig::_commandConfigWifi(String ssid, String pswd)
{
    sprintf(config_.ssid, "%s", ssid.c_str());
    sprintf(config_.pswd, "%s", pswd.c_str());
    config_.proto_mode = 0;
    _saveConfig(&config_);
    return true;
}

bool FishBotConfig::_commandConfigModeWifi(String serverIp, String serverPort)
{
    sprintf(config_.server_ip, "%s", serverIp.c_str());
    config_.server_port = serverPort.toInt();
    config_.proto_mode = 1;
    _saveConfig(&config_);
    return true;
}

bool FishBotConfig::_commandConfigModeSerial(String bautrate)
{
    config_.bautrate = bautrate.toInt();
    _saveConfig(&config_);
    return true;
}