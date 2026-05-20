#include <cmath>
#include <memory>
#include "control_node.hpp"

#include "limits"

ControlNode::ControlNode()
: Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  lookahead_distance_ = 2.0;   // meters
  linear_velocity_    = 0.5;   // m/s
  goal_tolerance_     = 0.5;   // meters

  path_received_ = false;
  odom_received_ = false;
  robot_x_   = 0.0;
  robot_y_   = 0.0;
  robot_yaw_ = 0.0;
  state_ = State::WAITING_FOR_PATH;

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10 Hz
    std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = *msg;
  path_received_ = true;

  if (current_path_.poses.empty()) {
    // Planner signals stop with empty path
    state_ = State::WAITING_FOR_PATH;
    stop();
  } else {
    state_ = State::FOLLOWING_PATH;
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_   = msg->pose.pose.position.x;
  robot_y_   = msg->pose.pose.position.y;
  robot_yaw_ = extractYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void ControlNode::timerCallback()
{
  if (!path_received_ || !odom_received_) return;
  if (state_ == State::WAITING_FOR_PATH) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_PATH;
    stop();
    return;
  }

  geometry_msgs::msg::Twist cmd = computeVelocity();
  cmd_vel_pub_->publish(cmd);
}

void ControlNode::stop()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = 0.0;
  cmd.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd);
}

bool ControlNode::goalReached()
{
  if (current_path_.poses.empty()) return true;

  // Check distance to last waypoint
  auto& last = current_path_.poses.back().pose.position;
  double dx = last.x - robot_x_;
  double dy = last.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

int ControlNode::findClosestWaypoint()
{
  int closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < static_cast<int>(current_path_.poses.size()); ++i) {
    double dx = current_path_.poses[i].pose.position.x - robot_x_;
    double dy = current_path_.poses[i].pose.position.y - robot_y_;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

geometry_msgs::msg::Point ControlNode::findLookaheadPoint(int closest_idx)
{
  // Walk forward from closest point until we find one lookahead_distance_ away
  for (int i = closest_idx; i < static_cast<int>(current_path_.poses.size()); ++i) {
    double dx = current_path_.poses[i].pose.position.x - robot_x_;
    double dy = current_path_.poses[i].pose.position.y - robot_y_;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= lookahead_distance_) {
      return current_path_.poses[i].pose.position;
    }
  }

  // If no point is far enough, just use the last waypoint
  return current_path_.poses.back().pose.position;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity()
{
  geometry_msgs::msg::Twist cmd;

  int closest_idx = findClosestWaypoint();
  geometry_msgs::msg::Point lookahead = findLookaheadPoint(closest_idx);

  // Transform lookahead point from global frame to robot frame
  double dx = lookahead.x - robot_x_;
  double dy = lookahead.y - robot_y_;
  double lx =  dx * std::cos(robot_yaw_) + dy * std::sin(robot_yaw_);
  double ly = -dx * std::sin(robot_yaw_) + dy * std::cos(robot_yaw_);

  // Pure pursuit curvature: k = 2*ly / (lx^2 + ly^2)
  double dist_sq = lx * lx + ly * ly;
  double curvature = 0.0;
  if (dist_sq > 1e-6) {
    curvature = 2.0 * ly / dist_sq;
  }

  cmd.linear.x  = linear_velocity_;
  cmd.angular.z = linear_velocity_ * curvature;

  return cmd;
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}