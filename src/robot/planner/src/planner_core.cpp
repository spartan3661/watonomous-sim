#include "planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace robot
{

    PlannerCore::PlannerCore(const rclcpp::Logger &logger)
        : logger_(logger)
    {
    }

    void PlannerCore::configure(
        double obstacle_threshold,
        double cost_penalty_scale)
    {
        obstacle_threshold_ = obstacle_threshold;
        cost_penalty_scale_ = cost_penalty_scale;

        if (cost_penalty_scale_ <= 0.0)
        {
            RCLCPP_WARN(
                logger_,
                "cost_penalty_scale must be positive. Resetting to 25.0.");
            cost_penalty_scale_ = 25.0;
        }
    }

    bool PlannerCore::createPath(
        const nav_msgs::msg::OccupancyGrid &map,
        const geometry_msgs::msg::Point &start,
        const geometry_msgs::msg::Point &goal,
        nav_msgs::msg::Path &path)
    {
        path = nav_msgs::msg::Path();
        path.header = map.header;

        CellIndex start_cell;
        CellIndex goal_cell;

        if (!worldToGrid(map, start, start_cell))
        {
            RCLCPP_WARN(logger_, "Start position is outside the map.");
            return false;
        }

        if (!worldToGrid(map, goal, goal_cell))
        {
            RCLCPP_WARN(logger_, "Goal position is outside the map.");
            return false;
        }

        if (!isTraversable(map, start_cell))
        {
            RCLCPP_WARN(logger_, "Start cell is blocked.");
            return false;
        }

        if (!isTraversable(map, goal_cell))
        {
            RCLCPP_WARN(logger_, "Goal cell is blocked.");
            return false;
        }

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_list;
        std::unordered_set<CellIndex, CellIndexHash> closed_list;
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
        std::unordered_map<CellIndex, double, CellIndexHash> g_score;

        auto getScore = [](
                            const std::unordered_map<CellIndex, double, CellIndexHash> &scores,
                            const CellIndex &index)
        {
            auto it = scores.find(index);
            if (it == scores.end())
            {
                return std::numeric_limits<double>::infinity();
            }
            return it->second;
        };

        g_score[start_cell] = 0.0;
        open_list.emplace(start_cell, heuristic(start_cell, goal_cell));

        bool found_path = false;

        while (!open_list.empty())
        {
            CellIndex current = open_list.top().index;
            open_list.pop();

            if (closed_list.find(current) != closed_list.end())
            {
                continue;
            }

            if (current == goal_cell)
            {
                found_path = true;
                break;
            }

            closed_list.insert(current);

            for (const auto &neighbor : getNeighbors(map, current))
            {
                if (closed_list.find(neighbor) != closed_list.end())
                {
                    continue;
                }

                const int cell_cost = getCellCost(map, neighbor);
                const double cost_penalty =
                    static_cast<double>(cell_cost) / cost_penalty_scale_;

                const double tentative_g =
                    getScore(g_score, current) +
                    movementCost(current, neighbor) +
                    cost_penalty;

                if (tentative_g < getScore(g_score, neighbor))
                {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;

                    const double f_score =
                        tentative_g + heuristic(neighbor, goal_cell);

                    open_list.emplace(neighbor, f_score);
                }
            }
        }

        if (!found_path)
        {
            RCLCPP_WARN(logger_, "A* could not find a valid path.");
            return false;
        }

        std::vector<CellIndex> cell_path;
        reconstructPath(came_from, start_cell, goal_cell, cell_path);

        if (cell_path.empty())
        {
            RCLCPP_WARN(logger_, "Path reconstruction returned an empty path.");
            return false;
        }

        for (const auto &cell : cell_path)
        {
            path.poses.push_back(gridToPose(map, cell));
        }

        RCLCPP_INFO(
            logger_,
            "A* generated path with %zu poses.",
            path.poses.size());

        return true;
    }

    bool PlannerCore::worldToGrid(
        const nav_msgs::msg::OccupancyGrid &map,
        const geometry_msgs::msg::Point &world,
        CellIndex &grid) const
    {
        const double origin_x = map.info.origin.position.x;
        const double origin_y = map.info.origin.position.y;
        const double resolution = map.info.resolution;

        if (resolution <= 0.0)
        {
            return false;
        }

        const double map_x = (world.x - origin_x) / resolution;
        const double map_y = (world.y - origin_y) / resolution;

        grid.x = static_cast<int>(std::floor(map_x));
        grid.y = static_cast<int>(std::floor(map_y));

        return isInsideMap(map, grid);
    }

    geometry_msgs::msg::PoseStamped PlannerCore::gridToPose(
        const nav_msgs::msg::OccupancyGrid &map,
        const CellIndex &grid) const
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = map.header;

        pose.pose.position.x =
            map.info.origin.position.x +
            (static_cast<double>(grid.x) + 0.5) * map.info.resolution;

        pose.pose.position.y =
            map.info.origin.position.y +
            (static_cast<double>(grid.y) + 0.5) * map.info.resolution;

        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        return pose;
    }

    bool PlannerCore::isInsideMap(
        const nav_msgs::msg::OccupancyGrid &map,
        const CellIndex &grid) const
    {
        return grid.x >= 0 &&
               grid.y >= 0 &&
               grid.x < static_cast<int>(map.info.width) &&
               grid.y < static_cast<int>(map.info.height);
    }

    int PlannerCore::getCellCost(
        const nav_msgs::msg::OccupancyGrid &map,
        const CellIndex &grid) const
    {
        if (!isInsideMap(map, grid))
        {
            return 100;
        }

        const int index =
            grid.y * static_cast<int>(map.info.width) + grid.x;

        const int value = static_cast<int>(map.data[index]);

        if (value < 0)
        {
            return 25;
        }

        return value;
    }

    bool PlannerCore::isTraversable(
        const nav_msgs::msg::OccupancyGrid &map,
        const CellIndex &grid) const
    {
        if (!isInsideMap(map, grid))
        {
            return false;
        }

        return static_cast<double>(getCellCost(map, grid)) <= obstacle_threshold_;
    }

    double PlannerCore::heuristic(
        const CellIndex &a,
        const CellIndex &b) const
    {
        const double dx = static_cast<double>(a.x - b.x);
        const double dy = static_cast<double>(a.y - b.y);
        return std::sqrt(dx * dx + dy * dy);
    }

    double PlannerCore::movementCost(
        const CellIndex &a,
        const CellIndex &b) const
    {
        const int dx = std::abs(a.x - b.x);
        const int dy = std::abs(a.y - b.y);

        if (dx == 1 && dy == 1)
        {
            return std::sqrt(2.0);
        }

        return 1.0;
    }

    std::vector<CellIndex> PlannerCore::getNeighbors(
        const nav_msgs::msg::OccupancyGrid &map,
        const CellIndex &current) const
    {
        std::vector<CellIndex> neighbors;
        neighbors.reserve(8);

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                if (dx == 0 && dy == 0)
                {
                    continue;
                }

                CellIndex neighbor(current.x + dx, current.y + dy);

                if (isTraversable(map, neighbor))
                {
                    neighbors.push_back(neighbor);
                }
            }
        }

        return neighbors;
    }

    void PlannerCore::reconstructPath(
        const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
        const CellIndex &start,
        const CellIndex &goal,
        std::vector<CellIndex> &result) const
    {
        result.clear();

        CellIndex current = goal;
        result.push_back(current);

        while (current != start)
        {
            auto it = came_from.find(current);

            if (it == came_from.end())
            {
                result.clear();
                return;
            }

            current = it->second;
            result.push_back(current);
        }

        std::reverse(result.begin(), result.end());
    }

} // namespace robot