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


double Control::normalizeAngleError(double error)
{

    if(error > M_PI) return -2*M_PI + error;

    if(error < -M_PI) return +2*M_PI + error;

    return error;

}

double Control::robotTargetDist(double* setpoint_xy, double* coords)
{
    return sqrt(pow(setpoint_xy[0] - coords[0]*100, 2) + pow(setpoint_xy[1] - coords[1]*100, 2));

}

bool Control::robotReachedTarget(double* setpoint_xy, double* coords, double bias)
{
    double dist = Control::robotTargetDist(setpoint_xy, coords);
    if(dist <= Odometry().wheelBaseDistanceM / 2 + bias) return true;

    return false;
}


void Control::setRobotMappingAngle(Robot* robot, double angle)
{
    double Kp = 5;
    double error = Control::normalizeAngleError(angle);
    double rotation_speed = Signal::saturate(Kp*error, M_PI/3, -M_PI/3);

    robot->setRotationSpeed(rotation_speed);
}


void Control::setRobotAngle(double* setpoint_xy, double* coords, Robot* robot)
{
    double Kp = 10;
    double error = Control::getAngleError(setpoint_xy,coords);
    error = Control::normalizeAngleError(error);
    double rotation_speed = Signal::saturate(Kp*error, M_PI/3, -M_PI/3);

    robot->setRotationSpeed(rotation_speed);

}

void Control::setRobotPosition(double* ref, double* coords, Robot* robot)
{
    double Kp = 10;
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


void Control::setpointRamp(double* setpoint_ramp, double* setpoint, double delta)
{
    if(setpoint[0] < 0 && setpoint_ramp[0] > setpoint[0])
    {
        setpoint_ramp[0] -= delta;
    }

    if(setpoint[1] < 0 && setpoint_ramp[1] > setpoint[1])
    {
        setpoint_ramp[1] -= delta;
    }

    if(setpoint[0] > 0 && setpoint_ramp[0] < setpoint[0])
    {
        setpoint_ramp[0] += delta;
    }

    if(setpoint[1] > 0 && setpoint_ramp[1] < setpoint[1])
    {
        setpoint_ramp[1] += delta;
    }
}
