#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "odometry.h"
#include "robot.h"

#include <algorithm>
#include <vector>

using namespace std;

struct Obstacle {
    double x;
    double y;
    double scan_angle;
    double scan_distance;

};

class Navigation
{
public :
    static void detectObstacles(LaserMeasurement* laserData, double* coords, vector<Obstacle>* obstacles);
    static bool detectColision(vector<Obstacle> obstacles, Robot* robot, double safedist);
    static double chooseShorterPath(vector<Obstacle>* obstacles, double* coords, double* setpoint);
    static vector<Obstacle>* queryObstacles(vector<Obstacle> obstacles, string str);
    static double exploreRadius(vector<Obstacle> obstacles, double* coords, double radius);
    static Obstacle queryMean(vector<Obstacle> obstacles);

};

#endif // NAVIGATION_H
