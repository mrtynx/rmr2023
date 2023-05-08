#include "odometry.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

#define M_PI 3.14159265358979323846

using namespace std;


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

    double x;
    double y;
    // ceknut lenr - lenL --> nan
    x = coords[0] + (Odometry().wheelBaseDistanceM*(lenL + lenR)) / (2*(lenR - lenL)) * (sin(coords[2]+deltaA) - sin(coords[2]));
    y = coords[1] - (Odometry().wheelBaseDistanceM*(lenL + lenR)) / (2*(lenR - lenL)) * (cos(coords[2]+deltaA) - cos(coords[2]));

    if(isnan(x) || isnan(y))
    {
        Odometry::curveLocalization(leftDiff, rightDiff, coords);
    }
    else
    {
        coords[0] = x;
        coords[1] = y;
    }

    coords[2] += deltaA;
}


double Odometry::cosd(double angle)
{
    return cos(angle * M_PI / 180.0);
}


double Odometry::sind(double angle)
{
    return sin(angle * M_PI / 180.0);
}


double Odometry::euclid2d(double* coords1, double* coords2)
{
    double dx = coords2[0] - coords1[0];
    double dy = coords2[1] - coords1[1];

    return sqrt(dx*dx + dy*dy);
}


bool Odometry::point_in_radius(pair<double, double> center, double radius, pair<double, double> point)
{
    double distance = sqrt(pow(point.first - center.first, 2) + pow(point.second - center.second, 2));

    return distance <= radius;
}




