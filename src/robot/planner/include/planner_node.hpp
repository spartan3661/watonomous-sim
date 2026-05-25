#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <mutex>
#include <string>

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode();

private:
  robot::PlannerCore planner_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex map_mutex_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  geometry_msgs::msg::PointStamped current_goal_;

  bool has_map_ = false;
  bool has_odom_ = false;
  bool goal_active_ = false;

  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_z_ = 0.0;


  std::string map_topic_;
  std::string goal_topic_;
  std::string odom_topic_;
  std::string path_topic_;

  double goal_tolerance_m_ = 0.03;
  double obstacle_threshold_ = 90.0;
  double cost_penalty_scale_ = 25.0;

  void loadParameters();

  void timerCallback();
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  bool planAndPublish();
  void resetGoalAndStopRobot();
  double distanceToGoal() const;
};

#endif