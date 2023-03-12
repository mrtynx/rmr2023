#include "control_system.h"
#include "odometry.h"
#include <math.h>
#include <iostream>

using namespace std;

double Control::getAngleError(double* setpoint_xy, double* coords)
{
    double setpoint_angle = atan2(setpoint_xy[1] - coords[1]*100, setpoint_xy[0] - coords[0]*100);
    return setpoint_angle - coords[2];
}


double Control::robotTargetDist(double* setpoint_xy, double* coords)
{
    return sqrt(pow(setpoint_xy[0] - coords[0]*100, 2) + pow(setpoint_xy[1] - coords[1]*100, 2));

}

bool Control::robotReachedTarget(double dist)
{
    if(dist <= Odometry().wheelBaseDistanceM *10 / 2) return true;

    return false;
}

void Control::rampRotationSpeed(double setpoint_angle, double angle)
{

}


void Control::setRobotAngle(double ref, double angle, Robot* robot)
{
    double Kp = 0.25;
    double error = ref - angle;
    double rotation_speed = Signal::saturate(Kp*error, M_PI/2, -M_PI/2);

    robot->setRotationSpeed(rotation_speed);

}

void Control::setRobotPosition(double* ref, double* coords, Robot* robot)
{
    double Kp = 0.25;
    double error = Control::robotTargetDist(ref, coords);
    double translation_speed = Signal::saturate(Kp*error, 350, -350);

    robot->setTranslationSpeed(translation_speed);

}

double Signal::saturate(double x, double upper, double lower)
{
    if(x < lower) return lower;

    if (x > upper) return upper;

    return x;
}
