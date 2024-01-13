#include <turtlebot/path_planner.h>

PathFinder::PathFinder(const GridGenerator& gridGenerator_): grid(gridGenerator_){};

/*
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

        // Explore neighbors (including diagonals)
        std::vector<std::pair<int, int>> neighbors = {
            {current.first + 1, current.second + 1}, {current.first - 1, current.second - 1},
            {current.first + 1, current.second - 1}, {current.first - 1, current.second + 1},
            {current.first + 1, current.second}, {current.first - 1, current.second},
            {current.first, current.second + 1}, {current.first, current.second - 1}
        };

        for (const auto& next : neighbors) {
            // Check if the neighbor is within grid bounds and not yet visited
            if (next.first >= 0 && next.first < grid.width_ && 
                next.second >= 0 && next.second < grid.height_ && 
                grid.grid_[next.second][next.first] == true &&
                cameFrom.find(next.first * grid.width_ + next.second) == cameFrom.end()) {
                
                q.push(next);
                cameFrom[next.first * grid.width_ + next.second] = current;
            }
        }


    }
    // If the destination is not reachable
    return std::vector<std::pair<double, double>>{};
    return path;
}*/
double PathFinder::movementCost(const std::pair<int, int>& current, const std::pair<int, int>& next) {
    // Define start position
    std::pair<int, int> start{grid.width_ / 2, grid.height_ / 2};

    // Check if the move is diagonal
    bool isDiagonal = (current.first != next.first) && (current.second != next.second);

    // Special case: If at start and move is diagonal, encourage this move
    if (current == start && isDiagonal) {
        return 0.5; // Lower cost for initial diagonal move
    }

    // Regular cost assignment
    if (isDiagonal) {
        return 1.1; // Higher cost for diagonal moves
    } else {
        return 1.0; // Lower cost for straight moves
    }
}

std::vector<std::pair<double, double>> PathFinder::findPath_BFS() {
    std::vector<std::pair<double, double>> path;
    std::priority_queue<std::pair<double, std::pair<int, int>>, std::vector<std::pair<double, std::pair<int, int>>>, std::greater<std::pair<double, std::pair<int, int>>>> pq;
    std::unordered_map<int, std::pair<int, int>> cameFrom;
    std::unordered_map<int, double> costSoFar;

    std::pair<int, int> start{grid.width_ / 2, grid.height_ / 2};

    pq.push({0, start});
    cameFrom[start.first * grid.width_ + start.second] = {-1, -1};
    costSoFar[start.first * grid.width_ + start.second] = 0;

    while (!pq.empty()) {
        std::pair<int, int> current = pq.top().second;
        pq.pop();

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

        std::vector<std::pair<int, int>> neighbors = {
            {current.first + 1, current.second + 1}, {current.first - 1, current.second - 1},
            {current.first + 1, current.second - 1}, {current.first - 1, current.second + 1},
            {current.first + 1, current.second}, {current.first - 1, current.second},
            {current.first, current.second + 1}, {current.first, current.second - 1}
        };

        for (const auto& next : neighbors) {
            double newCost = costSoFar[current.first * grid.width_ + current.second] + movementCost(current, next); // Implement movementCost to favor diagonals

            if (costSoFar.find(next.first * grid.width_ + next.second) == costSoFar.end() || newCost < costSoFar[next.first * grid.width_ + next.second]) {
                costSoFar[next.first * grid.width_ + next.second] = newCost;
                double priority = newCost;
                pq.push({priority, next});
                cameFrom[next.first * grid.width_ + next.second] = current;
            }
        }
    }

    // If the destination is not reachable
    return std::vector<std::pair<double, double>>{};
    return path;
}

void PathFinder::printPath(const std::vector<std::pair<double, double>>& path) {
    for (const auto& point : path) {
        std::cout << "(" << point.first << ", " << point.second << ") ";
    }
    std::cout << std::endl;
}

// Function to calculate area of the triangle formed by three points
double PathFinder::area(const std::pair<double, double>& A, 
            const std::pair<double, double>& B, 
            const std::pair<double, double>& C) {
    return std::abs((A.first*(B.second - C.second) + 
                     B.first*(C.second - A.second) + 
                     C.first*(A.second - B.second)) / 2.0);
}

// Function to find the furthest collinear point
std::pair<double, double> PathFinder::findFurthestCollinearPoint(const std::vector<std::pair<double, double>>& path) {
    if (path.size() < 2) {
        // If path has less than 2 points, return the only point or default
        return path.empty() ? std::make_pair(0.0, 0.0) : path.front();
    }

    std::pair<double, double> start = path.front();
    std::pair<double, double> furthestPoint = path[1];

    for (size_t i = 2; i < path.size(); ++i) {
        if (area(start, furthestPoint, path[i]) != 0) {
            // If the next point is not collinear, break the loop
            break;
        }
        furthestPoint = path[i];
    }

    return furthestPoint;
}