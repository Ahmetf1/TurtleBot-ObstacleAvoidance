#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <turtlebot/grid_generator.h>
#include <vector>
#include <utility> // for std::pair

class PathFinder {
public:
    PathFinder(const GridGenerator& GridGenerator_);

    double movementCost(const std::pair<int, int>& current, const std::pair<int, int>& next);
    std::vector<std::pair<double, double>> findPath_BFS();

    // Method to print the path
    void printPath(const std::vector<std::pair<double, double>>& path);

    // Function to find the furthest collinear point
    std::pair<double, double> findFurthestCollinearPoint(const std::vector<std::pair<double, double>>& path);

    // Function to calculate area of the triangle formed by three points
    double area(const std::pair<double, double>& A, 
            const std::pair<double, double>& B, 
            const std::pair<double, double>& C);

private:
    const GridGenerator& grid;


    // Helper methods for BFS
};

#endif // PATH_PLANNER_H