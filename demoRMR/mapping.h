#ifndef MAPPING_H
#define MAPPING_H

#include "odometry.h"
#include <math.h>
#include <limits.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;

class Mapping
{
public:
    static void mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath);
    static vector<vector<int>> mapAreaToGrid(char const *filePath);
    static vector<vector<int>> findPath(vector<vector<int>> &grid, pair<int, int> start, pair<int, int> target);
    static void gridToFile(vector<vector<int>> &grid, char const *filePath);
    static void print_grid(const std::vector<std::vector<int>>& grid);
};


#endif // MAPPING_H
