#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>
#include <iostream>
#include "Point.h"
#include "Motor.h"

class DistanceSensor {
    HardwareSerial& lidarSerial;
    int RX, TX;
    //String data = "0xA5 0x81";
    std::vector<uint8_t> data = {};

    std::vector<Point> points;
public:
    DistanceSensor(int RX, int TX, HardwareSerial& serial);
    void begin();
    void setBaudrate();
    void checkFirmwareVersion();
    bool disableDataOutput();
    bool enableDataOutput();
    bool setSampleFrequency();
    bool sendCommand(uint8_t *command, size_t length);
    Point receiveData();
    void readData();
    void collectData(Motor motor, int numRecords);
    std::vector<uint8_t> getData();
    std::vector<Point> getPoints();
    void resetData();
    void resetPoints();
    void addPoint();
};

#endif