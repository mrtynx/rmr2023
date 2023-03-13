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
    double angle = fmod(phi*(180.0 / M_PI) + 180.0, 360.0);

    if(phi < -M_PI) return angle + 180;

    return angle - 180.0;
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


void Odometry::circularLocalization(int leftDiff, int rightDiff, double* coords)
{
    double lenL = Odometry::getWheelDistance(leftDiff);
    double lenR = Odometry::getWheelDistance(rightDiff);
    double deltaA = (lenR-lenL)/Odometry().wheelBaseDistanceM;

    coords[0] += (Odometry().wheelBaseDistanceM*(lenL + lenR)) / (2*(lenR - lenL)) * (sin(coords[2]+deltaA) - sin(coords[2]));
    coords[1] -= (Odometry().wheelBaseDistanceM*(lenL + lenR)) / (2*(lenR - lenL)) * (cos(coords[2]+deltaA) - cos(coords[2]));
    coords[2] += deltaA;
}
