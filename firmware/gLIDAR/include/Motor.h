#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

//TFLUNA disc diameter = 52 mm
//Motor disc diameter = 45.25 mm

class Motor {
    int velocity = 500;
    double angle = 0;
    //const double angle_per_revolution = 413.7;
    const double angle_per_revolution = 422.5;
    const double angle_per_step = 0.9;
    int step_PIN;
    int dir_PIN;
    int en_PIN;
    int ms1_PIN;
    int ms2_PIN;
    bool full_revolution = false;
    int offset = 500;
public:
    Motor(int step_PIN, int dir_PIN, int en_PIN, int ms1_PIN, int ms2_PIN);
    double getAngle();
    void disable();
    void setDirection(bool dir);
    bool getDirection();
    void resetOffset();
    void setVelocity(int velocity);
    void rotate();
    void saveAngle();
    bool checkRevolution();
    void resetAngle();
};

#endif
