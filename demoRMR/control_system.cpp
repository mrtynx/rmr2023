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

    if(error > M_PI) return 2*M_PI + error;

    if(error < -M_PI) return -2*M_PI + error;

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

void Control::rampRotationSpeed(double setpoint_angle, double angle)
{

}


void Control::setRobotAngle(double* setpoint_xy, double* coords, Robot* robot)
{
    double Kp = 10;
    double error = Control::getAngleError(setpoint_xy,coords);
    error = Control::normalizeAngleError(error);
    double rotation_speed = Signal::saturate(Kp*error, M_PI/2, -M_PI/2);

    robot->setRotationSpeed(rotation_speed);

}

void Control::setRobotPosition(double* ref, double* coords, Robot* robot)
{
    double Kp = 10;
    double error = Control::robotTargetDist(ref, coords);
    double translation_speed = Signal::saturate(Kp*error, 350, -350);

    robot->setTranslationSpeed(translation_speed);

}

//void Control::setRobotArcPos(double* ref, double* coords, Robot* robot)
//{
//    double Kp_trans = 0.75;
//    double Kp_rot = 0.75;

//    double angle_error = Control::normalizeAngleError(Control::getAngleError(ref, coords));
//    double pos_error = Control::robotTargetDist(ref, coords);

//    robot->setArcSpeed(Kp_trans*pos_error, Kp_rot*angle_error);
//}

double Signal::saturate(double x, double upper, double lower)
{
    if(x < lower) return lower;

    if (x > upper) return upper;

    return x;
}
