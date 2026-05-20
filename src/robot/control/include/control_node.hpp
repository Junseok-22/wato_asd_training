#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
public:
  ControlNode();

private:
  robot::ControlCore control_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path current_path_;
  bool path_received_;
  bool odom_received_;

  double robot_x_;
  double robot_y_;
  double robot_yaw_;

  // Parameters
  double lookahead_distance_;
  double linear_velocity_;
  double goal_tolerance_;

  enum class State { WAITING_FOR_PATH, FOLLOWING_PATH, GOAL_REACHED };
  State state_;

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  void stop();
  bool goalReached();
  geometry_msgs::msg::Twist computeVelocity();
  int findClosestWaypoint();
  geometry_msgs::msg::Point findLookaheadPoint(int closest_idx);
  double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

#endif