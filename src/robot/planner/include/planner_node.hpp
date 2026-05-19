#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_core.hpp"
 
#include <vector>
#include <unordered_map>
#include <queue>
 
class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();
 
private:
  robot::PlannerCore planner_;
 
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
 
  geometry_msgs::msg::PointStamped goal_;
  bool goal_received_;
 
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  bool odom_received_;
  bool map_received_;
 
  enum class State { WAITING_FOR_GOAL, NAVIGATING };
  State state_;
 
  struct CellIndex {
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}
    bool operator==(const CellIndex & o) const { return x == o.x && y == o.y; }
    bool operator!=(const CellIndex & o) const { return !(*this == o); }
  };
 
  struct CellIndexHash {
    std::size_t operator()(const CellIndex & idx) const {
      return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
  };
 
  struct AStarNode {
    CellIndex index;
    double f_score;
    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
  };
 
  struct CompareF {
    bool operator()(const AStarNode & a, const AStarNode & b) {
      return a.f_score > b.f_score;
    }
  };
 
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();
 
  void planPath();
  bool goalReached();
  nav_msgs::msg::Path runAstar(double start_x, double start_y, double goal_x, double goal_y);
  nav_msgs::msg::Path reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
    const CellIndex & start, const CellIndex & goal) const;
 
  CellIndex worldToGrid(double wx, double wy) const;
  void gridToWorld(const CellIndex & idx, double & wx, double & wy) const;
  bool inBounds(const CellIndex & idx) const;
  bool isTraversable(const CellIndex & idx) const;
  double heuristic(const CellIndex & a, const CellIndex & b) const;
  double extractYaw(const geometry_msgs::msg::Quaternion & q);
};
 
#endif
 