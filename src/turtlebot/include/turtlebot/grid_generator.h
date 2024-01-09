#ifndef GRID_GENERATOR_H
#define GRID_GENERATOR_H

#include <vector>
#include <utility> // for std::pair
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <iostream>

class PathFinder;  // Forward declaration

class GridGenerator {
    friend class PathFinder;  // Make PathFinder a friend class
public:
    GridGenerator(double resolution);
    void setGridSize(int width, int height);
    void setObstacle(double x, double y);
    void setDestination(double x, double y);

    void adjustGridSizeForCoordinates(const std::vector<std::pair<double, double>>& points);

    // Method to check if a cell is an obstacle
    bool isObstacle(int x, int y) const;

    // Method to print the grid
    void printGrid() const;

private:
    double resolution_;
    int width_, height_;
    std::pair<int, int> min_bounds_;
    std::pair<int, int> destination_; // Grid coordinates
    std::pair<int, int> current_; // Current coordinates
    std::vector<std::vector<bool>> grid_; // false for obstacles

    // Convert real-world coordinates to grid coordinates
    std::pair<int, int> toGridCoordinates(double x, double y);
};

#endif // GRID_GENERATOR_H
