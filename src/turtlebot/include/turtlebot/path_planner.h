#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <turtlebot/grid_generator.h>
#include <vector>
#include <utility> // for std::pair

class PathFinder {
public:
    PathFinder(const GridGenerator& GridGenerator_);
    std::vector<std::pair<double, double>> findPath_BFS();

    // Method to print the path
    void printPath(const std::vector<std::pair<double, double>>& path);

private:
    const GridGenerator& grid;


    // Helper methods for BFS
};

#endif // PATH_PLANNER_H