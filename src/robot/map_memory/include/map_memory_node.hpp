#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"
#include <vector>

class MapMemoryNode : public rclcpp::Node {
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Global map
  nav_msgs::msg::OccupancyGrid global_map_;

  // Latest costmap
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool costmap_received_;

  // Robot position tracking
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  double last_update_x_;
  double last_update_y_;
  bool odom_received_;

  // Params
  double distance_threshold_;
  double resolution_;
  int width_;
  int height_;

  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
  void initializeGlobalMap();
  void integrateCostmap();
  void publishMap();
  double extractYaw(const geometry_msgs::msg::Quaternion& q);
};

#endif