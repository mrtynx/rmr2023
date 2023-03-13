#ifndef ODOMETRY_H
#define ODOMETRY_H
#define M_PI 3.14159265358979323846

    class Odometry
    {

        public:
            const long double tickToMeter = 0.000085292090497737556558;
            const long double wheelBaseDistanceM = 0.23;

            static double getWheelDistance(int diff);
            static int normalizeDiff(int diff);
            static void curveLocalization(int leftDiff, int rightDiff, double* coords);
            static void circularLocalization(int leftDiff, int rightDiff, double* coords);
            static double rad2deg(double phi);

    };



#endif // ODOMETRY_H
