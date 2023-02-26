#include "odometry.h"
#include <cmath>
#include <limits.h>

double Odometry::getWheelDistance(double diff)
{
    return Odometry().tickToMeter*diff;
}

double Odometry::normalizeDiff(double diff)
{
    if (diff > SHRT_MAX) return diff - USHRT_MAX;

    else if (diff < SHRT_MIN) return diff + USHRT_MAX;

    else return diff;
}

