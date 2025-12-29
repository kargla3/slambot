#include "Motor.h"

Motor::Motor(int step_PIN, int dir_PIN, int en_PIN, int ms1_PIN, int ms2_PIN)
{
    this->step_PIN = step_PIN;
    this->dir_PIN = dir_PIN;
    this->en_PIN = en_PIN;
    this->ms1_PIN = ms1_PIN;
    this->ms2_PIN = ms2_PIN;
    pinMode(step_PIN, OUTPUT);
    pinMode(dir_PIN, OUTPUT);
    pinMode(en_PIN, OUTPUT);
    pinMode(ms1_PIN, OUTPUT);
    pinMode(ms2_PIN, OUTPUT);

    digitalWrite(ms1_PIN, HIGH);
    digitalWrite(ms2_PIN, LOW);
}

double Motor::getAngle()
{
    return angle;
}

void Motor::disable()
{
    digitalWrite(en_PIN, HIGH);
}

void Motor::setDirection(bool dir)
{
    if (dir)
        digitalWrite(dir_PIN, HIGH);
    else
        digitalWrite(dir_PIN, LOW);
}

bool Motor::getDirection()
{
    return digitalRead(dir_PIN);
}

void Motor::resetOffset()
{
    offset = 500;
}

void Motor::setVelocity(int velocity)
{
    this->velocity = velocity;
}

void Motor::rotate()
{
    if (velocity >= 500)
    {
        digitalWrite(step_PIN, HIGH);
        delayMicroseconds(velocity);
        digitalWrite(step_PIN, LOW);
        delayMicroseconds(velocity);
    }
    else
    {
        digitalWrite(step_PIN, HIGH);
        delayMicroseconds(velocity + offset);
        digitalWrite(step_PIN, LOW);
        delayMicroseconds(velocity + offset);

        if (offset > 0)
            offset -= 1;
    }

    if (angle < angle_per_revolution)
    {
        angle += angle_per_step;
    }
    else
    {
        angle = 0;
    }

    if (angle == 0)
        full_revolution = true;
    else
        full_revolution = false;
}

bool Motor::checkRevolution()
{
    return full_revolution;
}

void Motor::resetAngle()
{
    this->angle = 0;
}