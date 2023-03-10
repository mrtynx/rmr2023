#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include "robot.h"

class Control
{
public:

    static double getAngleError(double* setpoint_xy, double* coords);
    static double robotTargetDist(double* setpoint_xy, double* coords);
    static double normalizeAngleError(double error);

    static bool robotReachedTarget(double* setpoint_xy, double* coords, double bias);

    static void rampRotationSpeed(double setpoint_angle, double angle);
    static void setRobotAngle(double* setpoint_xy, double* coords, Robot* robot);
    static void setRobotPosition(double* ref, double* coords, Robot* robot);
    static void setRobotArcPos(double* ref, double* coords, Robot* robot);
};


class Signal
{
public:
    static double saturate(double x, double upper, double lower);
};

#endif // CONTROL_SYSTEM_H
