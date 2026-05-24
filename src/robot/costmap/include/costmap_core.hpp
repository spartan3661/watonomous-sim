#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <vector>

namespace robot
{

    class CostmapCore
    {
    public:
        explicit CostmapCore(const rclcpp::Logger &logger);

        void updateFromScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        const std::vector<std::vector<int>> &getGrid() const;

        void initCostmap(
            double resolution,
            int width,
            int height,
            geometry_msgs::msg::Pose origin,
            double inflation_radius);
        nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

    private:
        void resetCostmap();
        void markObstaclesFromScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        bool worldToGrid(double x, double y, int &grid_x, int &grid_y);
        void inflateObstacles();

        rclcpp::Logger logger_;
        double inflation_radius_ = 2;
        int max_cost_ = 100;

        nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    };

}

#endif