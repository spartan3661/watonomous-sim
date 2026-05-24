#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <memory>
#include <mutex>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

namespace map_memory
{

class MapMemoryNode : public rclcpp::Node
{
public:
    MapMemoryNode();

private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();

    double quaternionToYaw(double x, double y, double z, double w);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<MapMemoryCore> core_;

    std::mutex mutex_;

    double robot_x_;
    double robot_y_;
    double robot_theta_;

    double last_robot_x_;
    double last_robot_y_;

    double update_distance_;
};

} // namespace map_memory

#endif