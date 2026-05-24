#include "control_core.hpp"

#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
: logger_(logger),
  lookahead_distance_(1.0),
  goal_tolerance_(0.1),
  linear_speed_(0.5),
  max_angular_speed_(1.5)
{
}

void ControlCore::setPath(const nav_msgs::msg::Path::SharedPtr path)
{
  current_path_ = path;

  if (!current_path_ || current_path_->poses.empty()) {
    RCLCPP_WARN(logger_, "Received an empty path.");
    return;
  }

  RCLCPP_INFO(
    logger_,
    "Received path with %zu poses.",
    current_path_->poses.size()
  );
}

void ControlCore::setOdometry(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_odom_ = odom;
}

geometry_msgs::msg::Twist ControlCore::computeVelocityCommand()
{
  if (!current_path_ || current_path_->poses.empty()) {
    return stopCommand();
  }

  if (!robot_odom_) {
    return stopCommand();
  }

  const geometry_msgs::msg::Point& robot_position =
    robot_odom_->pose.pose.position;

  const geometry_msgs::msg::Point& goal_position =
    current_path_->poses.back().pose.position;

  const double distance_to_goal =
    computeDistance(robot_position, goal_position);

  if (distance_to_goal <= goal_tolerance_) {
    RCLCPP_INFO_THROTTLE(
      logger_,
      *rclcpp::Clock::make_shared(),
      1000,
      "Goal reached. Stopping robot."
    );

    return stopCommand();
  }

    geometry_msgs::msg::PoseStamped lookahead_point;

    if (!findLookaheadPoint(lookahead_point)) {
    return stopCommand();
    }

    return computeVelocity(lookahead_point);
}
bool ControlCore::findLookaheadPoint(
  geometry_msgs::msg::PoseStamped& lookahead_point
) const
{
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    return false;
  }

  const geometry_msgs::msg::Point& robot_position =
    robot_odom_->pose.pose.position;

  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < current_path_->poses.size(); ++i) {
    const double distance = computeDistance(
      robot_position,
      current_path_->poses[i].pose.position
    );

    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  for (std::size_t i = closest_index; i < current_path_->poses.size(); ++i) {
    const double distance = computeDistance(
      robot_position,
      current_path_->poses[i].pose.position
    );

    if (distance >= lookahead_distance_) {
      lookahead_point = current_path_->poses[i];
      return true;
    }
  }

  lookahead_point = current_path_->poses.back();
  return true;
}
geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const geometry_msgs::msg::PoseStamped& target
) const
{
  geometry_msgs::msg::Twist cmd_vel;

  const geometry_msgs::msg::Point& robot_position =
    robot_odom_->pose.pose.position;

  const double robot_yaw =
    extractYaw(robot_odom_->pose.pose.orientation);

  const double dx = target.pose.position.x - robot_position.x;
  const double dy = target.pose.position.y - robot_position.y;

  const double target_heading = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(target_heading - robot_yaw);

  const double distance_to_target = std::max(
    computeDistance(robot_position, target.pose.position),
    0.001
  );

  // Pure Pursuit curvature:
  // curvature = 2 * sin(alpha) / lookahead_distance
  const double curvature =
    2.0 * std::sin(heading_error) / distance_to_target;

  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = linear_speed_ * curvature;

    if (cmd_vel.angular.z > max_angular_speed_) {
        cmd_vel.angular.z = max_angular_speed_;
    } else if (cmd_vel.angular.z < -max_angular_speed_) {
        cmd_vel.angular.z = -max_angular_speed_;
    }

  return cmd_vel;
}

geometry_msgs::msg::Twist ControlCore::stopCommand() const
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;
  cmd_vel.angular.z = 0.0;
  return cmd_vel;
}

double ControlCore::computeDistance(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b
) const
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;

  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(
  const geometry_msgs::msg::Quaternion& quat
) const
{
  const double siny_cosp =
    2.0 * (quat.w * quat.z + quat.x * quat.y);

  const double cosy_cosp =
    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

double ControlCore::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

}  