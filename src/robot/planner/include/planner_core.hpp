#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy)
  : x(xx), y(yy)
  {
  }

  CellIndex()
  : x(0), y(0)
  {
  }

  bool operator==(const CellIndex & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const CellIndex & other) const
  {
    return !(*this == other);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex & idx) const
  {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(const CellIndex & idx, double f)
  : index(idx), f_score(f)
  {
  }
};

struct CompareF
{
  bool operator()(const AStarNode & a, const AStarNode & b) const
  {
    return a.f_score > b.f_score;
  }
};

class PlannerCore
{
public:
  explicit PlannerCore(const rclcpp::Logger & logger);

  void configure(
    double obstacle_threshold,
    double cost_penalty_scale);

  bool createPath(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Point & start,
    const geometry_msgs::msg::Point & goal,
    nav_msgs::msg::Path & path);

private:
  rclcpp::Logger logger_;

  double obstacle_threshold_ = 90.0;
  double cost_penalty_scale_ = 25.0;

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Point & world,
    CellIndex & grid) const;

  geometry_msgs::msg::PoseStamped gridToPose(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & grid) const;

  bool isInsideMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & grid) const;

  int getCellCost(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & grid) const;

  bool isTraversable(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & grid) const;

  double heuristic(
    const CellIndex & a,
    const CellIndex & b) const;

  double movementCost(
    const CellIndex & a,
    const CellIndex & b) const;

  std::vector<CellIndex> getNeighbors(
    const nav_msgs::msg::OccupancyGrid & map,
    const CellIndex & current) const;

  void reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash> & came_from,
    const CellIndex & start,
    const CellIndex & goal,
    std::vector<CellIndex> & result) const;
};

}  // namespace robot

#endif