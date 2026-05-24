#include "costmap_core.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

    CostmapCore::CostmapCore(const rclcpp::Logger &logger)
        : logger_(logger),
          costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>())
    {
    }

    void CostmapCore::initCostmap(
        double resolution,
        int width,
        int height,
        geometry_msgs::msg::Pose origin,
        double inflation_radius)
    {
        costmap_data_->info.resolution = resolution;
        costmap_data_->info.width = width;
        costmap_data_->info.height = height;
        costmap_data_->info.origin = origin;

        costmap_data_->data.assign(width * height, 0);

        inflation_radius_ = inflation_radius;

        RCLCPP_INFO(
            logger_,
            "Costmap initialized: %d x %d cells, resolution %.2f, origin=(%.2f, %.2f), inflation radius %.2f",
            width,
            height,
            resolution,
            origin.position.x,
            origin.position.y,
            inflation_radius_);
    }

    void CostmapCore::updateFromScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        resetCostmap();
        markObstaclesFromScan(msg);
        inflateObstacles();
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const
    {
        return costmap_data_;
    }


    void CostmapCore::resetCostmap()
    {
        std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);
    }

    void CostmapCore::markObstaclesFromScan(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        double angle = msg->angle_min;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            const float range = msg->ranges[i];

            if (!std::isfinite(range) ||
                range < msg->range_min ||
                range > msg->range_max)
            {
                angle += msg->angle_increment;
                continue;
            }

            const double x = range * std::cos(angle);
            const double y = range * std::sin(angle);

            int grid_x = 0;
            int grid_y = 0;

            if (worldToGrid(x, y, grid_x, grid_y))
            {
                int index = grid_y * costmap_data_->info.width + grid_x;
                costmap_data_->data[index] = 100;
            }

            angle += msg->angle_increment;
        }
        RCLCPP_INFO(
            logger_,
            "LaserScan frame_id: %s",
            msg->header.frame_id.c_str());
    }

    bool CostmapCore::worldToGrid(double x, double y, int &grid_x, int &grid_y)
    {
        const double resolution = costmap_data_->info.resolution;
        const double origin_x = costmap_data_->info.origin.position.x;
        const double origin_y = costmap_data_->info.origin.position.y;
        const int width = static_cast<int>(costmap_data_->info.width);
        const int height = static_cast<int>(costmap_data_->info.height);

        grid_x = static_cast<int>(std::floor((x - origin_x) / resolution));
        grid_y = static_cast<int>(std::floor((y - origin_y) / resolution));

        if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
        {
            return false;
        }

        return true;
    }

    void CostmapCore::inflateObstacles()
    {
        std::vector<int8_t> inflated_grid = costmap_data_->data;

        const int width = static_cast<int>(costmap_data_->info.width);
        const int height = static_cast<int>(costmap_data_->info.height);
        const double resolution = costmap_data_->info.resolution;

        int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {

                int obstacle_index = y * width + x;

                if (costmap_data_->data[obstacle_index] != 100)
                {
                    continue;
                }

                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
                {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
                    {

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                        {
                            continue;
                        }

                        double distance = std::sqrt(dx * dx + dy * dy) * resolution;

                        if (distance > inflation_radius_)
                        {
                            continue;
                        }

                        int cost = static_cast<int>(
                            max_cost_ * (1.0 - distance / inflation_radius_));

                        int neighbor_index = ny * width + nx;

                        if (costmap_data_->data[neighbor_index] == 100)
                        {
                            inflated_grid[neighbor_index] = 100;
                            continue;
                        }

                        if (cost > inflated_grid[neighbor_index])
                        {
                            inflated_grid[neighbor_index] = static_cast<int8_t>(cost);
                        }
                    }
                }
            }
        }

        costmap_data_->data = inflated_grid;
    }

}