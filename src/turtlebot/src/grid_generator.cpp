#include <turtlebot/grid_generator.h>



GridGenerator::GridGenerator(double resolution) 
    : resolution_(resolution), width_(0), height_(0) {
}

void GridGenerator::setObstacle(double x, double y) {
    auto [grid_x, grid_y] = toGridCoordinates(x, y);
    if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
        grid_[grid_y][grid_x] = false;
    }
}

void GridGenerator::setGridSize(int width, int height) {
    width_ = width;
    height_ = height;
    grid_.clear();
    grid_.resize(height_, std::vector<bool>(width_, true));
}

void GridGenerator::setDestination(double x, double y) {
    destination_ = toGridCoordinates(x, y);
}


void GridGenerator::adjustGridSizeForCoordinates(const std::vector<std::pair<double, double>>& points) {
    double maxAbsValue_w = 0.0;
    double maxAbsValue_h = 0.0;
    for (const auto& point : points) {
        maxAbsValue_w = std::max(maxAbsValue_w, std::abs(point.first));
        maxAbsValue_h = std::max(maxAbsValue_h, std::abs(point.second));
    }

    int gridDimension_w = static_cast<int>(std::ceil(maxAbsValue_w / resolution_)) * 2 + 1; // +1 for the origin
    int gridDimension_h = static_cast<int>(std::ceil(maxAbsValue_h / resolution_)) * 2 + 1; // +1 for the origin
    setGridSize(gridDimension_w, gridDimension_h);
}

std::pair<int, int> GridGenerator::toGridCoordinates(double x, double y) {
    int grid_x = static_cast<int>(std::round((x / resolution_) + width_ / 2));
    int grid_y = static_cast<int>(std::round((y / resolution_) + height_ / 2));
    return {grid_x, grid_y};
}


bool GridGenerator::isObstacle(int x, int y) const{
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return grid_[y][x];
    }
    return false; // Out-of-bounds cells are treated as obstacles
}

void GridGenerator::printGrid() const {
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            std::cout << (grid_[y][x] ? '#' : '.');
        }
        std::cout << std::endl;
    }
}


