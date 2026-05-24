#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(robot::PlannerCore(this->get_logger()))
{
    loadParameters();

    planner_.configure(
        obstacle_threshold_,
        cost_penalty_scale_);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                       .reliable()
                       .transient_local();

    map_subscription_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            map_qos,
            std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_subscription_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            goal_topic_,
            10,
            std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_subscription_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            10,
            std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    path_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>(
            path_topic_,
            10);

    timer_ =
        this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PlannerNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Planner node started.");
}

void PlannerNode::loadParameters()
{
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("goal_topic", "/goal_point");
    this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
    this->declare_parameter<std::string>("path_topic", "/path");

    this->declare_parameter<double>("goal_tolerance", 0.30);
    this->declare_parameter<double>("plan_timeout_seconds", 10.0);

    this->declare_parameter<double>("obstacle_threshold", 90.0);
    this->declare_parameter<double>("cost_penalty_scale", 25.0);

    map_topic_ = this->get_parameter("map_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    path_topic_ = this->get_parameter("path_topic").as_string();

    goal_tolerance_m_ = this->get_parameter("goal_tolerance").as_double();
    plan_timeout_s_ = this->get_parameter("plan_timeout_seconds").as_double();

    obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
    cost_penalty_scale_ = this->get_parameter("cost_penalty_scale").as_double();
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = msg;
        has_map_ = true;
    }

    RCLCPP_INFO_ONCE(
        this->get_logger(),
        "Received occupancy grid.");

    if (!goal_active_)
    {
        return;
    }

    const double elapsed =
        (this->now() - goal_start_time_).seconds();

    if (elapsed > plan_timeout_s_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Map updated, but planning timeout has expired.");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Map updated. Replanning for active goal.");

    if (!planAndPublish())
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Replan failed after map update.");
    }
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{

    if (!has_map_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot accept goal yet: no map has been received.");
        return;
    }

    if (!has_odom_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Cannot accept goal yet: no odometry has been received.");
        return;
    }

    current_goal_ = *msg;
    goal_active_ = true;
    goal_start_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "Received goal: x=%.3f, y=%.3f",
        current_goal_.point.x,
        current_goal_.point.y);

    if (!planAndPublish())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Initial planning failed.");
        resetGoalAndStopRobot();
    }
}

void PlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_z_ = msg->pose.pose.position.z;

    has_odom_ = true;
}

void PlannerNode::timerCallback()
{
    if (!goal_active_)
    {
        return;
    }

    if (!has_odom_)
    {
        return;
    }

    const double elapsed =
        (this->now() - goal_start_time_).seconds();

    if (elapsed > plan_timeout_s_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Goal timed out after %.2f seconds.",
            elapsed);

        resetGoalAndStopRobot();
        return;
    }

    const double distance = distanceToGoal();

    if (distance <= goal_tolerance_m_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot reached goal. Distance: %.3f m. Elapsed time: %.2f s.",
            distance,
            elapsed);

        resetGoalAndStopRobot();
        return;
    }
}

bool PlannerNode::planAndPublish()
{
    if (!has_map_)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot plan: no map.");
        return false;
    }

    if (!has_odom_)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot plan: no odometry.");
        return false;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr map_copy;

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_copy = latest_map_;
    }

    if (!map_copy)
    {
        RCLCPP_WARN(this->get_logger(), "Cannot plan: map pointer is empty.");
        return false;
    }

    geometry_msgs::msg::Point start;
    start.x = robot_x_;
    start.y = robot_y_;
    start.z = robot_z_;

    geometry_msgs::msg::Point goal = current_goal_.point;

    nav_msgs::msg::Path planned_path;

    const bool success =
        planner_.createPath(
            *map_copy,
            start,
            goal,
            planned_path);

    if (!success)
    {
        return false;
    }

    planned_path.header.stamp = this->now();
    planned_path.header.frame_id = map_copy->header.frame_id;

    path_publisher_->publish(planned_path);

    RCLCPP_INFO(
        this->get_logger(),
        "Published path with %zu poses.",
        planned_path.poses.size());

    return true;
}

void PlannerNode::resetGoalAndStopRobot()
{
    goal_active_ = false;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->now();

    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        if (latest_map_)
        {
            empty_path.header.frame_id = latest_map_->header.frame_id;
        }
        else
        {
            empty_path.header.frame_id = "map";
        }
    }

    path_publisher_->publish(empty_path);

    RCLCPP_INFO(
        this->get_logger(),
        "Goal reset. Published empty path.");
}

double PlannerNode::distanceToGoal() const
{
    const double dx = robot_x_ - current_goal_.point.x;
    const double dy = robot_y_ - current_goal_.point.y;

    return std::sqrt(dx * dx + dy * dy);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}