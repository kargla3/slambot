#ifndef HALLSENSOR_H
#define HALLSENSOR_H

#include <Arduino.h>

class HallSensor {
    uint8_t H_PIN;
public:
    HallSensor(uint8_t H_PIN);
    int read();
};

#endif