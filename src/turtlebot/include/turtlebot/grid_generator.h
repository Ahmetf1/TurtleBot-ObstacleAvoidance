/*
Date:13/01/2024
Developed by: Musa Almaz
Project: Final Project
Summary: This header file is for generating the grid for the BSF path finding algorihm.
*/ 
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
    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Constructor for GridGenerator. Initializes the grid with a specified resolution.
    Input: resolution of the grid
    Output: no output
    Additional info: None
    */ 
    GridGenerator(double resolution);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Sets the size of the grid.
    Input: width and height of the grid
    Output: no output
    Additional info: Grid dimensions are specified in number of cells.
    */
    void setGridSize(int width, int height);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Marks a circular area in the grid as an obstacle based on real-world coordinates.
    Input: x and y coordinates in the real world, radius of the obstacle
    Output: no output
    Additional info: The function converts real-world coordinates to grid coordinates and marks the appropriate cells as obstacles.
    */
    void setObstacle(double x, double y, double obstacle_radius);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Sets the destination point on the grid.
    Input: x and y coordinates of the destination
    Output: no output
    Additional info: Converts real-world coordinates of the destination to grid coordinates.
    */
    void setDestination(double x, double y);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Adjusts the size of the grid based on a set of real-world coordinate points.
    Input: vector of real-world coordinate points
    Output: no output
    Additional info: This can be used to resize the grid dynamically based on the range of coordinates.
    */
    void adjustGridSizeForCoordinates(const std::vector<std::pair<double, double>>& points);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Checks if a specified cell is an obstacle.
    Input: x and y coordinates of the cell in the grid
    Output: boolean value indicating if the cell is an obstacle
    Additional info: Useful for pathfinding algorithms to determine accessible paths.
    */
    bool isObstacle(int x, int y) const;

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Prints the current state of the grid to the console.
    Input: no input
    Output: no output (console output)
    Additional info: Useful for debugging and visualizing the grid layout.
    "." for the obstacle, "#" for the free points, "C" is the current coordinate(orijin) and "D" for the destination
    */
    void printGrid() const;

private:
    // Resolution of the grid
    double resolution_;
    // Size of the grid
    int width_, height_;
    std::pair<int, int> min_bounds_;
    std::pair<int, int> destination_; // Grid coordinates
    std::pair<int, int> current_; // Current coordinates
    std::vector<std::vector<bool>> grid_; // false for obstacles

    // Convert real-world coordinates to grid coordinates
    std::pair<int, int> toGridCoordinates(double x, double y);
};

#endif // GRID_GENERATOR_H
