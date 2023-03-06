#include "odometry.h"
#include <math.h>
#include <limits.h>
#include <iostream>

#define M_PI 3.14159265358979323846


double Odometry::getWheelDistance(int diff)
{
    return Odometry().tickToMeter*diff;
}

int Odometry::normalizeDiff(int diff)
{
    if (diff > SHRT_MAX) return diff - USHRT_MAX;

    else if (diff < SHRT_MIN) return diff + USHRT_MAX;

    else return diff;
}


double Odometry::rad2deg(double phi)
{
    return fmod(phi*(180.0 / M_PI) + 180.0, 360.0) - 180.0;
}

void Odometry::curveLocalization(int leftDiff, int rightDiff, double* coords)
{
    double lenL = Odometry::getWheelDistance(leftDiff);
    double lenR = Odometry::getWheelDistance(rightDiff);
    double deltaA = (lenR-lenL)/Odometry().wheelBaseDistanceM;

    coords[0] += ((lenL + lenR) / 2) * cos(coords[2]);
    coords[1] += ((lenL + lenR) / 2) * sin(coords[2]);
    coords[2] += deltaA;
}
