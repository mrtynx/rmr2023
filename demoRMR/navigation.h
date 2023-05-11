#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "odometry.h"
#include "control_system.h".h"
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

enum ObstacleType
{
    RightWall,
    LeftWall,
    FrontWall,
    Clear,
    UNKNWN
};

enum RobotOrientation
{
    FacingUP,
    FacingDOWN,
    FacingLEFT,
    FacingRIGHT
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
    static ObstacleType determineObstacleType(vector<Obstacle> obstacles);
    static bool alignToWall(ObstacleType obstacle_type, double robot_angle);
    static RobotOrientation determineRobotOrientation(double* coords);
    static void generateSetpoint(double* temp_setpoint, double* coords, ObstacleType obstacle_type, RobotOrientation robot_orientation, Obstacle query_mean, double safedist);

};

#endif // NAVIGATION_H
