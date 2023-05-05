#include "mapping.h"
#include "odometry.h"
#include <queue>
#include <utility>
#include <vector>
#include <tuple>
#include <fstream>
#include <iostream>

using namespace std;

vector<vector<int>> Mapping::mapAreaToGrid(char const *filePath)
{
    std::vector<int> x, y;
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

    for (int i = 0; i < y_max + 1; i++) {
        for (int j = 0; j < x_max + 1; j++) {
            std::cout << grid[i][j] << " ";
        }
        std::cout << std::endl;
    }


    vector<vector<int>> grid_vec(y_max + 1, vector<int>(x_max + 1, 0));
    for(int i=0; i<y_max; i++)
    {
        for(int j=0; j<x_max; j++)
        {
            grid_vec[i][j] = grid[i][j];
        }
    }

    return grid_vec;
}


void Mapping::mapAreaToFile(LaserMeasurement* laserData, double* coords, char const *filePath)
{
    std::ofstream file(filePath, std::ios_base::app);
    if(file.is_open())
    {
        for(int i=0; i<= laserData->numberOfScans; i++)
        {
            double sd = laserData->Data[i].scanDistance;
            if(sd > 130 && sd < 3000 && !(640> sd && sd > 700))
            {
                double x = coords[0]*100 + laserData->Data[i].scanDistance * Odometry::cosd(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
                double y = coords[1]*100 + laserData->Data[i].scanDistance * Odometry::sind(coords[2]*180/M_PI - laserData->Data[i].scanAngle)/10;
                file<<round(x/10)<<","<<round(y/10)<<"\n";
            }

        }
        file.close();
    }
}



vector<vector<int>> Mapping::findPath(vector<vector<int>> &grid, pair<int, int> start, pair<int, int> target)
{
    int rows = grid.size();
    int cols = grid[0].size();

    grid[target.first][target.second] = 2;

    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<vector<pair<int, int>>> previous(rows, vector<pair<int, int>>(cols, {-1, -1}));

    queue<pair<int, int>> q;

    int dr[] = {-1, 0, 1, 0};
    int dc[] = {0, 1, 0, -1};

    visited[start.first][start.second] = true;
    q.push(start);

    while (!q.empty()) {
        pair<int, int> current = q.front();
        q.pop();

        if (current == target) {
            break;
        }

        for (int i = 0; i < 4; i++) {
            int newRow = current.first + dr[i];
            int newCol = current.second + dc[i];

            if (newRow >= 0 && newRow < rows && newCol >= 0 && newCol < cols && !visited[newRow][newCol]) {
                int cell = grid[newRow][newCol];
                if (cell == 0 || cell == 2) {
                    visited[newRow][newCol] = true;
                    q.push({newRow, newCol});
                    previous[newRow][newCol] = current;
                }
            }
        }
    }

    vector<vector<int>> path;
    if (visited[target.first][target.second]) {
        for (pair<int, int> current = target; current != start; current = previous[current.first][current.second]) {
            path.push_back({current.first, current.second});
        }
        path.push_back({start.first, start.second});
        reverse(path.begin(), path.end());
    }

    return path;
}


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


void Mapping::print_grid(const std::vector<std::vector<int>>& grid)
{
    for (const auto& row : grid) {
        for (const auto& element : row) {
            std::cout << element << " ";
        }
        std::cout << std::endl;
    }
}


