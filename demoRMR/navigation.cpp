#include "navigation.h"
#include "mapping.h"
#include "odometry.h"
#include <utility>
#include <vector>
#include <iostream>
#include <cassert>
#include <string>


using namespace std;

vector<Obstacle>* Navigation::queryObstacles(vector<Obstacle> obstacles, string str)
{
    if(obstacles.empty())
    {
        return nullptr;
    }

    double min_angle, max_angle, lidar_min_dist, lidar_max_dist;
    vector<Obstacle>* query_output = new vector<Obstacle>;

    if(str == "right_narrow")
    {
        min_angle = 60;
        max_angle = 120;
        lidar_min_dist = 13;
        lidar_max_dist = 60;
    }
    else if(str == "left_narrow")
    {
        min_angle = 250;
        max_angle = 310;
        lidar_min_dist = 13;
        lidar_max_dist = 60;
    }
    else if(str == "right_wide")
    {
        min_angle = 0;
        max_angle = 180;
        lidar_min_dist = 13;
        lidar_max_dist = 250;
    }
    else if(str == "left_wide")
    {
        min_angle = 180;
        max_angle = 360;
        lidar_min_dist = 13;
        lidar_max_dist = 250;
    }
    else if(str == "front_wide")
    {
        min_angle = 90;
        max_angle = 270;
        lidar_min_dist = 13;
        lidar_max_dist = 250;
    }
    else if(str == "front_narrow")
    {
        min_angle = 5;
        max_angle = 355;
        lidar_min_dist = 13;
        lidar_max_dist = 250;
    }
    else if(str == "full")
    {
        min_angle = 0;
        max_angle = 360;
        lidar_min_dist = 13;
        lidar_max_dist = 250;
    }
    else if(str == "full_short")
    {
        min_angle = 0;
        max_angle = 360;
        lidar_min_dist = 13;
        lidar_max_dist = 70;
    }
    else
    {
        return nullptr;
    }


    for (Obstacle& obstacle : obstacles) {
        bool in_valid_distance = (obstacle.scan_distance > lidar_min_dist) && (obstacle.scan_distance < lidar_max_dist);
        bool in_valid_angle = (obstacle.scan_angle >= min_angle) && (obstacle.scan_angle <= max_angle);
        bool front_wide_condition = !((360 - obstacle.scan_angle >= min_angle) && (360 - obstacle.scan_angle <= max_angle));



        if(in_valid_distance)
        {
            if((str == "front_wide") && front_wide_condition)
            {
                query_output->push_back(obstacle);
            }
            else if((str == "front_narrow") && !in_valid_angle)
            {
                query_output->push_back(obstacle);
            }
            else if((str != "front_wide") && (str != "front_narrow") && in_valid_angle)
            {
                query_output->push_back(obstacle);
            }
        }
    }

    return query_output;

}

void Navigation::detectObstacles(LaserMeasurement* laserData, double* coords, vector<Obstacle>* obstacles)
{

    for(int i=0; i<= laserData->numberOfScans; i++)
    {
        double dist = laserData->Data[i].scanDistance;
        double angle = laserData->Data[i].scanAngle;

        if((dist > 130) && (dist < 2500))
//        if(((dist > 130) && (dist < 750)) && !((360-angle >= 90) && (360-angle <= 270)))
        {
            double x = coords[0]*100 + dist * Odometry::cosd(coords[2]*180/M_PI - angle)/10;
            double y = coords[1]*100 + dist * Odometry::sind(coords[2]*180/M_PI - angle)/10;

            Obstacle obstacle;
            obstacle.x = x;
            obstacle.y = y;
            obstacle.scan_angle = angle;
            obstacle.scan_distance = dist/10;

            obstacles->push_back(obstacle);

        }

    }

//    cout<<obstacles->size()<<endl;
}




bool Navigation::detectColision(vector<Obstacle> obstacles, Robot* robot, double safedist)
{

    if(!obstacles.empty())
    {
        double min_dist = obstacles[0].scan_distance;
        for(Obstacle& obstacle: obstacles)
        {
            if(obstacle.scan_distance < min_dist)
            {
                min_dist = obstacle.scan_distance;
            }
        }

        if(min_dist <= safedist)
        {
            robot->setTranslationSpeed(0);
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }

}


double Navigation::chooseShorterPath(vector<Obstacle>* obstacles, double* coords, double* setpoint)
{
    size_t window_size = 30;
    double radius = 10;
    double max_points = 0;
    double avg_x = 0;
    double avg_y = 0;
    double points = 0;
    vector<Obstacle> buffer;

    if(!obstacles->empty() && (window_size > obstacles->size()))
    {
        for(size_t i = window_size - 1; i < obstacles->size()-1; i++)
        {

            avg_x = 0;
            avg_y = 0;
            buffer.clear();
            points = 0;

            for(size_t j = 0; j < window_size -1; j++)
            {
                buffer.push_back(obstacles->at(i-j));
                avg_x += obstacles->at(i-j).x / window_size;
                avg_y += obstacles->at(i-j).y / window_size;
            }

            for(size_t j = 0; j < buffer.size(); j++)
            {
                if(Odometry::point_in_radius(make_pair(avg_x, avg_y), radius, make_pair(buffer.at(j).x, buffer.at(j).y)))
                {
                    points += 1;
                }
            }

            if(points > max_points)
            {
                max_points = points;
            }

        }

        return max_points;

    }
    else
    {
        return 0;
    }


}


Obstacle Navigation::queryMean(vector<Obstacle> obstacles)
{
    Obstacle mean;
    for(Obstacle& obstacle: obstacles)
    {
        mean.x += obstacle.x / obstacles.size();
        mean.y += obstacle.y / obstacles.size();
        mean.scan_angle += obstacle.scan_angle / obstacles.size();
        mean.scan_distance += obstacle.scan_distance /obstacles.size();
    }

    return mean;
}


ObstacleType Navigation::determineObstacleType(vector<Obstacle> obstacles)
{
    vector<Obstacle>* front_query = Navigation::queryObstacles(obstacles, "front_narrow");
    vector<Obstacle>* left_query = Navigation::queryObstacles(obstacles, "left_narrow");
    vector<Obstacle>* right_query = Navigation::queryObstacles(obstacles, "right_narrow");

    if(left_query->size() > right_query->size())
    {
        return LeftWall;
    }
    else if(left_query->size() < right_query->size())
    {
        return RightWall;
    }
    else if((!front_query->empty()) && (right_query->empty()) && (left_query->empty()))
    {
        return FrontWall;
    }
    else if(front_query->empty())
    {
        return Clear;
    }
    else
    {
        return UNKNWN;
    }

}


RobotOrientation Navigation::determineRobotOrientation(double* coords)
{
    double angle = Odometry::rad2deg(coords[2]);

    if(((angle >= -180) && (angle <= -135)) || ((angle >= 135) && (angle <= 180)))
    {
        return FacingLEFT;
    }
    else if(((angle >= -45) && (angle <= -1)) || ((angle >= 0) && (angle <= 45)))
    {
        return FacingRIGHT;
    }
    else if((angle > 45) && (angle < 135))
    {
        return FacingUP;
    }
    else
    {
        return FacingDOWN;
    }

}


void Navigation::generateSetpoint(double* temp_setpoint, double* coords, ObstacleType obstacle_type, RobotOrientation robot_orientation, Obstacle query_mean, double safedist)
{
    if(obstacle_type == LeftWall)
    {
        switch (robot_orientation)
        {
        case FacingLEFT:
            temp_setpoint[0] = coords[0]*100 - 10;
            temp_setpoint[1] = coords[1]*100 + (safedist - query_mean.scan_distance);
            break;
        case FacingRIGHT:
            temp_setpoint[0] = coords[0]*100 + 10;
            temp_setpoint[1] = coords[1]*100 - (safedist - query_mean.scan_distance);
            break;
        case FacingUP:
            temp_setpoint[0] = coords[0]*100 + (safedist - query_mean.scan_distance);;
            temp_setpoint[1] = coords[1]*100 + 10;
            break;
        case FacingDOWN:
            temp_setpoint[0] = coords[0]*100 - (safedist - query_mean.scan_distance);
            temp_setpoint[1] = coords[1]*100 - 10;
            break;
        }
    }
}


