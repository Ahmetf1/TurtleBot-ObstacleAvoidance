#include <turtlebot/path_planner.h>

PathFinder::PathFinder(const GridGenerator& gridGenerator_): grid(gridGenerator_){};

std::vector<std::pair<double, double>> PathFinder::findPath_BFS() {
    std::vector<std::pair<double, double>> path;
    std::queue<std::pair<int, int>> q;
    std::unordered_map<int, std::pair<int, int>> cameFrom;

    std::pair<int, int> start{grid.width_ / 2, grid.height_ / 2};

    q.push(start);
    cameFrom[start.first * grid.width_ + start.second] = {-1, -1};

    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        q.pop();

        if (current == grid.destination_) {
            while (current != start) {
                // Convert back to world coordinates
                double worldX = (current.first - grid.width_ / 2) * grid.resolution_;
                double worldY = (current.second - grid.height_ / 2) * grid.resolution_;
                path.push_back({worldX, worldY});
                current = cameFrom[current.first * grid.width_ + current.second];
            }
            // Convert the start point back to world coordinates
            path.push_back({grid.current_.first, grid.current_.second});
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Explore neighbors
        std::vector<std::pair<int, int>> neighbors = {
            {current.first + 1, current.second}, {current.first - 1, current.second},
            {current.first, current.second + 1}, {current.first, current.second - 1}
        };

        for (const auto& next : neighbors) {
            // Corrected check: The cell should be traversable (true in your case)
            if (next.first >= 0 && next.first < grid.width_ && 
                next.second >= 0 && next.second < grid.height_&& 
                grid.grid_[next.second][next.first] == true && // Corrected condition
                cameFrom.find(next.first * grid.width_ + next.second) == cameFrom.end()) {
                q.push(next);
                cameFrom[next.first * grid.width_ + next.second] = current;
            }
        }
    }
    // If the destination is not reachable
    // return std::vector<std::pair<double, double>>{};
    return path;
}

void PathFinder::printPath(const std::vector<std::pair<double, double>>& path) {
    for (const auto& point : path) {
        std::cout << "(" << point.first << ", " << point.second << ") ";
    }
    std::cout << std::endl;
}