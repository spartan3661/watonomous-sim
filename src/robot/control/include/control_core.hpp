#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void setPath(const nav_msgs::msg::Path::SharedPtr path);
    void setOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);

    geometry_msgs::msg::Twist computeVelocityCommand();

  private:
    bool findLookaheadPoint(geometry_msgs::msg::PoseStamped& lookahead_point) const;
    geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped& target
    ) const;

    geometry_msgs::msg::Twist stopCommand() const;

    double computeDistance(
      const geometry_msgs::msg::Point& a,
      const geometry_msgs::msg::Point& b
    ) const;

    double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;
    double normalizeAngle(double angle) const;

    rclcpp::Logger logger_;

    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_speed_;
};

} 

#endif