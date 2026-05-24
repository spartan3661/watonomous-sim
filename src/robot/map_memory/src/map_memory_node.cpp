#include "map_memory_node.hpp"

#include <chrono>
#include <memory>
#include <functional>
#include <limits>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

namespace map_memory
{

    MapMemoryNode::MapMemoryNode()
        : Node("map_memory_node")
    {
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_theta_ = 0.0;
        last_robot_x_ = std::numeric_limits<double>::quiet_NaN();
        last_robot_y_ = std::numeric_limits<double>::quiet_NaN();

        const double resolution = this->declare_parameter<double>("resolution", 0.05);
        const int width = this->declare_parameter<int>("width", 800);
        const int height = this->declare_parameter<int>("height", 800);
        const double origin_x = this->declare_parameter<double>("origin_x", -20.0);
        const double origin_y = this->declare_parameter<double>("origin_y", -20.0);
        const std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");

        update_distance_ =
            this->declare_parameter<double>("movement_threshold", 0.9);

        const double update_period_sec =
            this->declare_parameter<double>("update_period_sec", 1.0);

        core_ = std::make_unique<MapMemoryCore>(
            resolution,
            width,
            height,
            origin_x,
            origin_y,
            frame_id);

        costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/costmap",
            rclcpp::QoS(10),
            std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered",
            rclcpp::QoS(10),
            std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(10).transient_local());

        const auto update_period =
            std::chrono::duration<double>(update_period_sec);

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(update_period),
            std::bind(&MapMemoryNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Map memory node started.");
    }

    void MapMemoryNode::costmapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        const auto now = this->now();

        const bool first_update = std::isnan(last_robot_x_);

        const double dist =
            first_update ? std::numeric_limits<double>::infinity()
                         : std::hypot(robot_x_ - last_robot_x_,
                                      robot_y_ - last_robot_y_);

        const bool moved_enough = dist >= update_distance_;

        static rclcpp::Time last_map_update_time = now;
        const bool time_elapsed =
            first_update || (now - last_map_update_time).seconds() >= 0.1;

        if (first_update || moved_enough || time_elapsed)
        {
            core_->updateMap(msg, robot_x_, robot_y_, robot_theta_);

            last_robot_x_ = robot_x_;
            last_robot_y_ = robot_y_;
            last_map_update_time = now;
        }
    }
    /*


    void MapMemoryNode::costmapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        RCLCPP_WARN(
            this->get_logger(),
            "costmapCallback fired: robot=(%.2f, %.2f, %.2f)",
            robot_x_,
            robot_y_,
            robot_theta_);

        core_->updateMap(msg, robot_x_, robot_y_, robot_theta_);

        last_robot_x_ = robot_x_;
        last_robot_y_ = robot_y_;
    }



    */

    void MapMemoryNode::odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;

        robot_theta_ = quaternionToYaw(qx, qy, qz, qw);
    }

    void MapMemoryNode::timerCallback()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        nav_msgs::msg::OccupancyGrid map_msg = *core_->getMapData();
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "sim_world";

        map_pub_->publish(map_msg);
    }

    double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w)
    {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

} // namespace map_memory

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_memory::MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}