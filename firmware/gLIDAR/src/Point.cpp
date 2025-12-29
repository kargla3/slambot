#include "Point.h"

Point::Point() : angle(0.0), distance(0), strength(0) {}

void Point::setAngle(double angle) {
    this->angle = angle;
}

void Point::setDistance(int distance) {
    this->distance = distance;
}

void Point::setStrength(int strength) {
    this->strength = strength;
}

int Point::getDistance() {
    return distance;
}

int Point::getStrength() {
    return strength;
}

double Point::getAngle() {
    return angle;
}