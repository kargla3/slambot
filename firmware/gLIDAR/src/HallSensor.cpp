#include "HallSensor.h"

HallSensor::HallSensor(uint8_t H_PIN) {
    pinMode(H_PIN, INPUT);
}

int HallSensor::read() {
    return analogRead(H_PIN);
}