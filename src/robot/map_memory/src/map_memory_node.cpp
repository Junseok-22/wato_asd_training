#include <cmath>
#include <memory>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  resolution_ = 0.05;         // 5cm per cell (finer than costmap)
  width_ = 2000;              // 2000 cells = 100m x 100m
  height_ = 2000;
  distance_threshold_ = 1.5;  // update every 1.5m

  costmap_received_ = false;
  odom_received_ = false;
  robot_x_ = 0.0;
  robot_y_ = 0.0;
  robot_yaw_ = 0.0;
  last_update_x_ = 0.0;
  last_update_y_ = 0.0;

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::timerCallback, this));

  initializeGlobalMap();
}

void MapMemoryNode::initializeGlobalMap()
{
  global_map_.info.resolution = resolution_;
  global_map_.info.width = width_;
  global_map_.info.height = height_;

  // Origin at center of map
  global_map_.info.origin.position.x = -(width_ / 2) * resolution_;
  global_map_.info.origin.position.y = -(height_ / 2) * resolution_;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  global_map_.header.frame_id = "odom";
  global_map_.data.assign(width_ * height_, -1); // -1 = unknown
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = extractYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void MapMemoryNode::timerCallback()
{
  if (!costmap_received_ || !odom_received_) return;

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist >= distance_threshold_) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    integrateCostmap();
    publishMap();
  }
}

void MapMemoryNode::integrateCostmap()
{
  int costmap_width = latest_costmap_.info.width;
  int costmap_height = latest_costmap_.info.height;
  double costmap_res = latest_costmap_.info.resolution;

  int map_center_x = width_ / 2;
  int map_center_y = height_ / 2;

  for (int row = 0; row < costmap_height; ++row) {
    for (int col = 0; col < costmap_width; ++col) {
      int8_t cost = latest_costmap_.data[row * costmap_width + col];
      if (cost < 0) continue; // skip unknown cells

      // Costmap local position (relative to robot)
      double local_x = (col - costmap_width / 2) * costmap_res;
      double local_y = (row - costmap_height / 2) * costmap_res;

      // Rotate by robot yaw, then translate to global frame
      double global_x = robot_x_ + local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
      double global_y = robot_y_ + local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

      // Convert global position to map grid indices
      int mx = static_cast<int>(std::round(global_x / resolution_)) + map_center_x;
      int my = static_cast<int>(std::round(global_y / resolution_)) + map_center_y;

      if (mx < 0 || mx >= width_ || my < 0 || my >= height_) continue;

      // Overwrite if new cost is higher, or cell was unknown
      int idx = my * width_ + mx;
      if (global_map_.data[idx] == -1 || cost > global_map_.data[idx]) {
        global_map_.data[idx] = cost;
      }
    }
  }
}

void MapMemoryNode::publishMap()
{
  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

double MapMemoryNode::extractYaw(const geometry_msgs::msg::Quaternion& q)
{
  // yaw from quaternion: atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}