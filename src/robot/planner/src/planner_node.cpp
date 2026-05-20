#include <cmath>
#include <memory>
#include <algorithm>
#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  goal_received_ = false;
  map_received_  = false;
  odom_received_ = false;
  robot_x_   = 0.0;
  robot_y_   = 0.0;
  robot_yaw_ = 0.0;
  state_ = State::WAITING_FOR_GOAL;

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_  = *msg;
  map_received_ = true;

  // Replan whenever the map updates so the path avoids new obstacles
  if (state_ == State::NAVIGATING) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_          = *msg;
  goal_received_ = true;
  state_         = State::NAVIGATING;

  RCLCPP_INFO(this->get_logger(),
    "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);

  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_      = msg->pose.pose.position.x;
  robot_y_      = msg->pose.pose.position.y;
  robot_yaw_    = extractYaw(msg->pose.pose.orientation);
  odom_received_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::NAVIGATING) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;

    // Publish empty path so the control node stops the robot
    nav_msgs::msg::Path empty;
    empty.header.stamp    = this->get_clock()->now();
    empty.header.frame_id = "odom";
    path_pub_->publish(empty);
  }
}

void PlannerNode::planPath()
{
  if (!map_received_ || !goal_received_ || !odom_received_) return;

  nav_msgs::msg::Path path = runAstar(
    robot_x_, robot_y_,
    goal_.point.x, goal_.point.y);

  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "A*: no path found");
    return;
  }

  path.header.stamp    = this->get_clock()->now();
  path.header.frame_id = "odom";
  path_pub_->publish(path);

  RCLCPP_INFO(this->get_logger(), "Path published: %zu waypoints", path.poses.size());
}

bool PlannerNode::goalReached()
{
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

nav_msgs::msg::Path PlannerNode::runAstar(
  double start_x, double start_y,
  double goal_x,  double goal_y)
{
  CellIndex start = worldToGrid(start_x, start_y);
  CellIndex goal  = worldToGrid(goal_x,  goal_y);

  if (!inBounds(start)) {
    RCLCPP_WARN(this->get_logger(), "A*: start is outside map");
    return {};
  }
  if (!inBounds(goal)) {
    RCLCPP_WARN(this->get_logger(), "A*: goal is outside map");
    return {};
  }

  // g_score[n] = cheapest cost from start to n found so far
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  g_score[start] = 0.0;

  // came_from[n] = node preceding n on the best known path
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  // Open set: min-heap ordered by f = g + h
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  open_set.emplace(start, heuristic(start, goal));

  // Closed set: already fully evaluated nodes
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  // 8-directional moves: {dx, dy, move_cost}
  const std::vector<std::tuple<int, int, double>> neighbours = {
    { 1,  0, 1.0},   {-1,  0, 1.0},   { 0,  1, 1.0},   { 0, -1, 1.0},
    { 1,  1, 1.414}, {-1,  1, 1.414}, { 1, -1, 1.414}, {-1, -1, 1.414}
  };

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (closed.count(current.index)) continue;
    closed[current.index] = true;

    if (current.index == goal) {
      return reconstructPath(came_from, start, goal);
    }

    for (auto & [dx, dy, move_cost] : neighbours) {
      CellIndex nb(current.index.x + dx, current.index.y + dy);

      if (!inBounds(nb))      continue;
      if (!isTraversable(nb)) continue;
      if (closed.count(nb))   continue;

      // Add the cell's inflation cost (0-100) normalised to 0-1
      int map_idx      = nb.y * static_cast<int>(current_map_.info.width) + nb.x;
      double cell_cost = static_cast<double>(current_map_.data[map_idx]) / 100.0;

      double tentative_g = g_score[current.index] + move_cost + cell_cost;

      if (!g_score.count(nb) || tentative_g < g_score[nb]) {
        g_score[nb]   = tentative_g;
        came_from[nb] = current.index;
        open_set.emplace(nb, tentative_g + heuristic(nb, goal));
      }
    }
  }

  RCLCPP_WARN(this->get_logger(), "A*: no path found");
  return {};
}

nav_msgs::msg::Path PlannerNode::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
  const CellIndex & start,
  const CellIndex & goal) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "odom";

  std::vector<CellIndex> cells;
  CellIndex current = goal;
  while (current != start) {
    cells.push_back(current);
    current = came_from.at(current);
  }
  cells.push_back(start);
  std::reverse(cells.begin(), cells.end());

  for (auto & cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    gridToWorld(cell, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z    = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  return path;
}

PlannerNode::CellIndex PlannerNode::worldToGrid(double wx, double wy) const
{
  double ox  = current_map_.info.origin.position.x;
  double oy  = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  return CellIndex(
    static_cast<int>(std::floor((wx - ox) / res)),
    static_cast<int>(std::floor((wy - oy) / res)));
}

void PlannerNode::gridToWorld(const CellIndex & idx, double & wx, double & wy) const
{
  double ox  = current_map_.info.origin.position.x;
  double oy  = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;
  wx = ox + (idx.x + 0.5) * res;
  wy = oy + (idx.y + 0.5) * res;
}

bool PlannerNode::inBounds(const CellIndex & idx) const
{
  return idx.x >= 0 &&
         idx.y >= 0 &&
         idx.x < static_cast<int>(current_map_.info.width) &&
         idx.y < static_cast<int>(current_map_.info.height);
}

bool PlannerNode::isTraversable(const CellIndex & idx) const
{
  int i = idx.y * static_cast<int>(current_map_.info.width) + idx.x;
 
  return current_map_.data[i] <=0;
}

double PlannerNode::heuristic(const CellIndex & a, const CellIndex & b) const
{
  double dx = static_cast<double>(a.x - b.x);
  double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerNode::extractYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}