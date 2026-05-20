#include <cmath>
#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  resolution_ = 0.1;      
  width_ = 200;             
  height_ = 200;
  inflation_radius_ = 1.5;

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap()
{
  // Fill entire grid with 0 (free space)
  occupancy_grid_.assign(width_ * height_, 0);
}

void CostmapNode::markObstacle(int gx, int gy)
{
  // Bounds check
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) return;

  occupancy_grid_[gy * width_ + gx] = 100;
}

void CostmapNode::inflateObstacles()
{
  // Work on a copy so obstacle cells don't inflate each other
  std::vector<int8_t> inflated = occupancy_grid_;

  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {

      // Only inflate from obstacle cells
      if (occupancy_grid_[row * width_ + col] != 100) continue;

      for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
        for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
          int nr = row + dr;
          int nc = col + dc;

          if (nr < 0 || nr >= height_ || nc < 0 || nc >= width_) continue;

          double dist = std::sqrt(dr * dr + dc * dc) * resolution_;
          if (dist > inflation_radius_) continue;

          // Linear cost falloff from 100 at obstacle to 0 at inflation radius
          int8_t cost = static_cast<int8_t>(100.0 * (1.0 - dist / inflation_radius_));

          // Only overwrite if this cost is higher than what's already there
          if (cost > inflated[nr * width_ + nc]) {
            inflated[nr * width_ + nc] = cost;
          }
        }
      }
    }
  }

  occupancy_grid_ = inflated;
}

void CostmapNode::publishCostmap(const std_msgs::msg::Header& header)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.header.frame_id = "base_link";

  msg.info.resolution = resolution_;
  msg.info.width = width_;
  msg.info.height = height_;

  // Place origin so robot is at center of grid
  msg.info.origin.position.x = -(width_ / 2) * resolution_;
  msg.info.origin.position.y = -(height_ / 2) * resolution_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data = occupancy_grid_;

  costmap_pub_->publish(msg);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  initializeCostmap();

  int center_x = width_ / 2;
  int center_y = height_ / 2;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double range = scan->ranges[i];

    // Skip invalid readings
    if (range < scan->range_min || range > scan->range_max) continue;

    double angle = scan->angle_min + i * scan->angle_increment;

    // Polar → Cartesian
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    // Cartesian → grid indices
    int gx = static_cast<int>(std::round(x / resolution_)) + center_x;
    int gy = static_cast<int>(std::round(y / resolution_)) + center_y;

    markObstacle(gx, gy);
  }

  inflateObstacles();
  publishCostmap(scan->header);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}