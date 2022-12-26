#include "Arduino.h"

class Rate
{
private:
    uint64_t last_loop_time_;
    float rate_;
    uint16_t period_;

public:
    Rate(float rate);
    void sleep();
    ~Rate() = default;
};


