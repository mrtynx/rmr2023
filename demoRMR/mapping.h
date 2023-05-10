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

enum Direction {
    UP, DOWN, LEFT, RIGHT
};

class Mapping
{
public:
    static void mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath);
    static void mapAreaToVec(LaserMeasurement* laserData, double* coords, vector<pair<int, int>>* map_vec);
    static vector<vector<int>> VecMapToGrid(vector<pair<int,int>> map_vec);
    static vector<vector<int>> mapAreaToGrid(char const *filePath);
    static void gridToFile(vector<vector<int>> &grid, char const *filePath);
    static void print_grid(const std::vector<std::vector<int>>& grid);
    static vector<vector<int>> gridFromFile(char const *filePath);
    static vector<vector<int>> enlargeObstacles(const vector<vector<int>>& grid);

    static bool isValid(vector<vector<int>>& grid, int row, int col);
    static vector<vector<int>> floodFill(vector<vector<int>> grid, int start_x, int start_y, int target_x, int target_y);
    static vector<pair<int,int>> getPath(vector<vector<int>> grid, int start_x, int start_y);
    static void printPath(vector<vector<int>> grid, vector<pair<int,int>> path);
    static vector<pair<int,int>> trimPath(vector<pair<int,int>> path);
    static Direction getDirection(std::pair<int, int> a, std::pair<int, int> b);
};


#endif // MAPPING_H
