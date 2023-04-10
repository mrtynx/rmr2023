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

void Odometry::mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath)
{
    std::ofstream file(filePath, std::ios_base::app);
    if(file.is_open())
    {
        for(int i=0; i<= laserData->numberOfScans; i++)
        {
            double x = coords[0]*100 + laserData->Data[i].scanDistance * Odometry::cosd(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
            double y = coords[1]*100 + laserData->Data[i].scanDistance * Odometry::sind(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
            file<<x<<","<<y<<"\n";
        }
        file.close();
    }
}

void Odometry::mapAreaToGrid(char const *filePath)
{
    std::vector<double> x, y;
    std::ifstream file(filePath);
    if (file.is_open()) {
        std::string line;
        while (getline(file, line)) {
            std::stringstream ss(line);
            double val1, val2;
            char comma;
            ss >> val1 >> comma >> val2;
            x.push_back(val1);
            y.push_back(val2);
        }
        file.close();
    } else {
        std::cerr << "Error: could not open file " << filePath << std::endl;
    }

    double x_max = *std::max_element(x.begin(), x.end());
    double x_min = *std::min_element(x.begin(), x.end());

    double y_max = *std::max_element(y.begin(), y.end());
    double y_min = *std::min_element(y.begin(), y.end());


}

