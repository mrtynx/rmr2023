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

    static void setRobotAngle(double* setpoint_xy, double* coords, Robot* robot);
    static void setRobotPosition(double* ref, double* coords, Robot* robot);
    static void setRobotMappingAngle(Robot* robot, double angle);
    static void setpointRamp(double* setpoint_ramp, double* setpoint, double delta);
};


class Signal
{
public:
    static double saturate(double x, double upper, double lower);
};

#endif // CONTROL_SYSTEM_H
