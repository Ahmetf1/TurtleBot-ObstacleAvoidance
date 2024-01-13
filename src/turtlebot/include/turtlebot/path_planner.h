/*
Date:13/01/2024
Developed by: Musa Almaz
Project: Final Project
Summary: This header file is for generating a path with BSF algorithm.
*/ 
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <turtlebot/grid_generator.h>
#include <vector>
#include <utility> // for std::pair

class PathFinder {
public:
    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Constructor for PathFinder. Initializes with a reference to a GridGenerator instance.
    Input: Reference to a GridGenerator instance
    Output: no output
    Additional info: This sets up the PathFinder with the grid to be used for pathfinding.
    */
    PathFinder(const GridGenerator& GridGenerator_);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Calculates the cost of moving from the current cell to the next cell.
    Input: The current and next grid cells as pairs of integers
    Output: Cost as a double
    Additional info: This function can be tailored to favor certain types of movements .
    At first step, the diagonal movement is desired, then the straight movement, so the cost is arranged for this.
    */
    double movementCost(const std::pair<int, int>& current, const std::pair<int, int>& next);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Finds the shortest path from the start to the destination using the BFS algorithm.
    Input: no input
    Output: Vector of points (doubles) representing the path
    Additional info: The path is represented in world coordinates.
    */
    std::vector<std::pair<double, double>> findPath_BFS();

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Prints the path to the console.
    Input: A vector of points representing the path
    Output: no output (console output)
    Additional info: Useful for debugging and visualizing the computed path.
    */
    void printPath(const std::vector<std::pair<double, double>>& path);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Finds the furthest collinear point from the start in a path.
    Input: A vector of points representing the path
    Output: A pair of doubles representing the furthest collinear point
    Additional info: Useful in path optimization to reduce the number of waypoints.
    */
    std::pair<double, double> findFurthestCollinearPoint(const std::vector<std::pair<double, double>>& path);

    /*
    Date: 13.01.2024
    Developed by: Musa Almaz
    Summary: Calculates the area of a triangle formed by three points.
    Input: Three points represented as pairs of doubles
    Output: Area of the triangle as a double
    Additional info: Used in algorithms like finding collinear points.
    */
    double area(const std::pair<double, double>& A, 
                const std::pair<double, double>& B, 
                const std::pair<double, double>& C);

private:
    const GridGenerator& grid; // Reference to the grid used for pathfinding

};

#endif // PATH_PLANNER_H
