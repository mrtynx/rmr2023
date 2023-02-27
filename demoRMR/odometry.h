#ifndef ODOMETRY_H
#define ODOMETRY_H

    class Odometry
    {

        public:
            const long double tickToMeter = 0.000085292090497737556558;
            const long double wheelBaseDistanceM = 0.23;

            static double getWheelDistance(double diff);
            static double normalizeDiff(double diff);
            static void curveLocalization(double leftDiff, double rightDiff, double* coords);
            static double rad2deg(double phi);

    };



#endif // ODOMETRY_H
