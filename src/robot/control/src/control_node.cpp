#include "control_node.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

ControlNode::ControlNode()
: Node("control"),
  control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path",
    10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  );

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel",
    10
  );

  control_timer_ = this->create_wall_timer(
    100ms,
    std::bind(&ControlNode::controlTimerCallback, this)
  );
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  control_.setPath(msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  control_.setOdometry(msg);
}

void ControlNode::controlTimerCallback()
{
  geometry_msgs::msg::Twist cmd_vel = control_.computeVelocityCommand();
  cmd_vel_pub_->publish(cmd_vel);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}