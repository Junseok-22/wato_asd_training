#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
#include <vector>


class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  double resolution_;
  int width_;
  int height_;
  double inflation_radius_;

  std::vector<int8_t> occupancy_grid_;

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void initializeCostmap();
  void markObstacle(int gx, int gy);
  void inflateObstacles();
  void publishCostmap(const std_msgs::msg::Header& header);
};

#endif