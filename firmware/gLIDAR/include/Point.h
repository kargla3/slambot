#ifndef POINT_H
#define POINT_H

class Point {
    int distance;
    int strength;
    double angle = 0;

public: 
    Point();
    void setDistance(int distance);
    void setStrength(int strength);
    void setAngle(double angle);

    int getDistance();
    int getStrength();
    double getAngle();
};

#endif