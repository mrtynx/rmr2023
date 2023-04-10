#ifndef ODOMETRY_H
#define ODOMETRY_H
#define M_PI 3.14159265358979323846

#include "rplidar.h"
#include <fstream>

    class Odometry
    {

        public:
            const long double tickToMeter = 0.000085292090497737556558;
            const long double wheelBaseDistanceM = 0.23;

            static double getWheelDistance(int diff);
            static double cosd(double angle);
            static double sind(double angle);
            static int normalizeDiff(int diff);
            static void curveLocalization(int leftDiff, int rightDiff, double* coords);
            static void circularLocalization(int leftDiff, int rightDiff, double* coords);
            static void mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath);
            static void mapAreaToGrid(char const *filePath);
            static double rad2deg(double phi);

    };



#endif // ODOMETRY_H
