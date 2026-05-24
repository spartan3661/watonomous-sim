#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
#include <string>

class CostmapNode : public rclcpp::Node
{
public:
    CostmapNode();

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void processParameters();

private:
    robot::CostmapCore costmap_;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    std::string laserscan_topic_;
    std::string costmap_topic_;

    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
};

#endif