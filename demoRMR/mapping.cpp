#include "mapping.h"
#include "odometry.h"
#include <queue>
#include <utility>
#include <vector>
#include <tuple>
#include <fstream>
#include <iostream>

using namespace std;

//vector<vector<int>> Mapping::mapAreaToGrid(char const *filePath)
//{
//    std::vector<int> x, y;
//    std::ifstream file(filePath);
//    if (file.is_open()) {
//        std::string line;
//        while (getline(file, line)) {
//            std::stringstream ss(line);
//            double val1, val2;
//            char comma;
//            ss >> val1 >> comma >> val2;
//            x.push_back(val1);
//            y.push_back(val2);
//        }
//        file.close();
//    } else {
//        std::cerr << "Error: could not open file " << filePath << std::endl;
//    }


//    double x_min = *std::min_element(x.begin(), x.end());
//    double y_min = *std::min_element(y.begin(), y.end());

//    if(x.size() != y.size())
//    {
//        std::cout<<"Vector X and Y must have same size!";
//    }
//    for(int i = 0; i < x.size(); i++)
//    {
//        x[i] -= x_min;
//        y[i] -= y_min;
//    }

//    int y_max =  *std::max_element(y.begin(), y.end());
//    int x_max =  *std::max_element(x.begin(), x.end());

//    std::cout<<y_max<<","<<x_max<<","<<x_min<<","<<y_min<<"\n";


//    int** grid = (int**)std::calloc((y_max + 1) , sizeof(int*));
//    for (int i = 0; i < (y_max + 1); i++)
//    {
//        grid[i] = (int*)std::calloc((x_max + 1) , sizeof(int));
//    }


//    for(int i=0; i < x.size(); i++)
//    {
//        grid[y_max - y[i]][x[i]] = 1;
//    }



//    vector<vector<int>> grid_vec(y_max + 1, vector<int>(x_max + 1, 0));
//    for(int i=0; i<y_max; i++)
//    {
//        for(int j=0; j<x_max; j++)
//        {
//            grid_vec[i][j] = grid[i][j];
//        }
//    }

//    for(int j=0;j<x_max;j++)
//    {
//        grid_vec[y_max][j] = 1;
//    }

//    return grid_vec;
//}


//void Mapping::mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath)
//{
//    std::ofstream file(filePath, std::ios_base::app);
//    if(file.is_open())
//    {
//        for(int i=0; i<= laserData->numberOfScans; i++)
//        {
//            double sd = laserData->Data[i].scanDistance;
//            if(sd > 130 && sd < 3000 && !(640> sd && sd > 700))
//            {
//                double x = coords[0]*100 + laserData->Data[i].scanDistance * Odometry::cosd(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
//                double y = coords[1]*100 + laserData->Data[i].scanDistance * Odometry::sind(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
//                file<<round(x/10)<<","<<round(y/10)<<"\n";
//            }

//        }
//        file.close();
//    }
//}


void Mapping::gridToFile(vector<vector<int>> &grid, char const *filePath)
{
    std::ofstream file(filePath);

    for (const auto& row : grid) {
        for (size_t i = 0; i < row.size(); i++) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << std::endl;
    }

    file.close();

}



void Mapping::mapAreaToVec(LaserMeasurement* laserData, double* coords, vector<pair<int, int>>* map_vec)
{
    for(int i=0; i<= laserData->numberOfScans; i++)
    {
        double sd = laserData->Data[i].scanDistance;
        if(sd > 130 && sd < 3000 && !(640> sd && sd > 700))
        {
            double x = coords[0]*100 + laserData->Data[i].scanDistance * Odometry::cosd(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
            double y = coords[1]*100 + laserData->Data[i].scanDistance * Odometry::sind(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;

            map_vec->push_back(make_pair(round(x/10), round(y/10)));
        }

    }
}


vector<vector<int>> Mapping::VecMapToGrid(vector<pair<int,int>> map_vec)
{
    vector<int> x, y;

    for(const auto& pair: map_vec)
    {
        x.push_back(pair.first);
        y.push_back(pair.second);
    }

    double x_min = *std::min_element(x.begin(), x.end());
    double y_min = *std::min_element(y.begin(), y.end());

    if(x.size() != y.size())
    {
        std::cout<<"Vector X and Y must have same size!";
    }
    for(int i = 0; i < x.size(); i++)
    {
        x[i] -= x_min;
        y[i] -= y_min;
    }

    int y_max =  *std::max_element(y.begin(), y.end());
    int x_max =  *std::max_element(x.begin(), x.end());

    std::cout<<y_max<<","<<x_max<<","<<x_min<<","<<y_min<<"\n";


    int** grid = (int**)std::calloc((y_max + 1) , sizeof(int*));
    for (int i = 0; i < (y_max + 1); i++)
    {
        grid[i] = (int*)std::calloc((x_max + 1) , sizeof(int));
    }


    for(int i=0; i < x.size(); i++)
    {
        grid[y_max - y[i]][x[i]] = 1;
    }



    vector<vector<int>> grid_vec(y_max + 1, vector<int>(x_max + 1, 0));
    for(int i=0; i<y_max; i++)
    {
        for(int j=0; j<x_max; j++)
        {
            grid_vec[i][j] = grid[i][j];
        }
    }

    for(int j=0;j<x_max;j++)
    {
        grid_vec[y_max][j] = 1;
    }

    return grid_vec;
}



void Mapping::print_grid(const std::vector<std::vector<int>>& grid)
{
    for (const auto& row : grid) {
        for (const auto& element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}



vector<vector<int>> Mapping::gridFromFile(char const *filePath)
{
    ifstream file(filePath);
    vector<vector<int>> data;
    string line;
    while (getline(file, line)) {
        vector<int> row;
        stringstream ss(line);
        string cell;
        while (getline(ss, cell, ',')) {
            row.push_back(stoi(cell));
        }
        data.push_back(row);
    }
    return data;
}



bool Mapping::isValid(vector<vector<int>>& grid, int y, int x) {
    return ((y >= 0) && (y < grid.size()) && (x >= 0) && (x < grid[0].size())) ;
}


vector<vector<int>> Mapping::floodFill(vector<vector<int>> grid, int start_x, int start_y, int target_x, int target_y)
{
    grid[target_y][target_x] = 2;

    int dy[] = {-1, 0, 1, 0};
    int dx[] = {0, 1, 0, -1};

    vector<pair<int,int>> coords;

    pair<int,int> actual_coord = make_pair(target_y, target_x);

    coords.push_back(actual_coord);


    while(!coords.empty())
    {
        for(int i=0; i < 4; i++)
        {
            int new_y = actual_coord.first + dy[i];
            int new_x = actual_coord.second + dx[i];
            if(Mapping::isValid(grid, new_y, new_x))
            {
                if((grid[new_y][new_x] == 0) && (grid[actual_coord.first][actual_coord.second] != 1))
                {
                    grid[new_y][new_x] = grid[actual_coord.first][actual_coord.second] + 1;
                    coords.push_back(make_pair(new_y, new_x));
                }

            }
        }

        auto it = find(coords.begin(), coords.end(), actual_coord);
        if (it != coords.end())
        {
            coords.erase(it);
        }
        if(!coords.empty())
        {

            actual_coord = coords.front();
        }



    }

    Mapping::print_grid(grid);
    return grid;

}


vector<pair<int,int>> Mapping::getPath(vector<vector<int>> grid, int start_x, int start_y)
{
    vector<pair<int,int>> path;
    pair<int,int> curr_point = make_pair(start_y, start_x);
    int point_val = grid[start_y][start_x];
    path.push_back(curr_point);

    int dy[] = {-1, 0, 1, 0};
    int dx[] = {0, 1, 0, -1};

    while(point_val != 2)
    {
        for(int i=0; i<4; i++)
        {
            int new_y = curr_point.first + dy[i];
            int new_x = curr_point.second + dx[i];
            if(Mapping::isValid(grid, new_y, new_x))
            {
                if((grid[new_y][new_x] != 1) && (grid[new_y][new_x] < grid[curr_point.first][curr_point.second]))
                {
                    curr_point.first = new_y;
                    curr_point.second = new_x;
                }
            }
        }

        path.push_back(curr_point);
        point_val = grid[curr_point.first][curr_point.second];
    }

    return path;
}

void Mapping::printPath(vector<vector<int>> grid, vector<pair<int,int>> path)
{
    for (const auto& p : path) {
        grid[p.first][p.second] = 5;
    }

    Mapping::print_grid(grid);
}



vector<vector<int>> Mapping::enlargeObstacles(const vector<vector<int>>& grid) {
    int numRows = grid.size();
    int numCols = grid[0].size();

    vector<vector<int>> enlargedGrid(numRows, vector<int>(numCols, 0));

    for (int row = 0; row < numRows; row++) {
        for (int col = 0; col < numCols; col++) {
            if (grid[row][col] == 1) {
                // Set the corresponding cell and its adjacent cells to be obstacles
                for (int i = row - 3; i <= row + 3; i++) {
                    for (int j = col - 2; j <= col + 2; j++) {
                        if (i >= 0 && i < numRows && j >= 0 && j < numCols) {
                            enlargedGrid[i][j] = 1;
                        }
                    }
                }
            }
        }
    }

    return enlargedGrid;
}

Direction Mapping::getDirection(std::pair<int, int> a, std::pair<int, int> b) {
    if (b.first > a.first) {
        return DOWN;
    } else if (b.first < a.first) {
        return UP;
    } else if (b.second > a.second) {
        return RIGHT;
    } else {
        return LEFT;
    }
}


std::vector<std::pair<int, int>> Mapping::trimPath(const std::vector<std::pair<int, int>> path) {
    if (path.size() < 3) {
        return path; // If there are less than 3 points, just return the original path
    }

    std::vector<std::pair<int, int>> trimmedPath;
    trimmedPath.push_back(path[0]); // Add the first point to the path

    // Iterate over the path points, checking the direction between every 3 consecutive points
    for (int i = 1; i < path.size() - 1; ++i) {
        Direction direction1 = Mapping::getDirection(path[i-1], path[i]);
        Direction direction2 = Mapping::getDirection(path[i], path[i+1]);
        if (direction1 != direction2) {
            trimmedPath.push_back(path[i]); // If the direction changes, add the middle point to the path
        }
    }

    trimmedPath.push_back(path.back()); // Add the last point to the path

    return trimmedPath;
}
