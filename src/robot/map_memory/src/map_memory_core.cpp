#include "map_memory_core.hpp"

#include <algorithm>
#include <cmath>

namespace map_memory
{

MapMemoryCore::MapMemoryCore(
    double resolution,
    int width,
    int height,
    double origin_x,
    double origin_y,
    const std::string & frame_id)
: global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
  resolution_(resolution),
  width_(width),
  height_(height),
  origin_x_(origin_x),
  origin_y_(origin_y),
  frame_id_(frame_id)
{
    initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap()
{
    global_map_->header.frame_id = frame_id_;

    global_map_->info.resolution = resolution_;
    global_map_->info.width = width_;
    global_map_->info.height = height_;

    global_map_->info.origin.position.x = origin_x_;
    global_map_->info.origin.position.y = origin_y_;
    global_map_->info.origin.position.z = 0.0;

    global_map_->info.origin.orientation.x = 0.0;
    global_map_->info.origin.orientation.y = 0.0;
    global_map_->info.origin.orientation.z = 0.0;
    global_map_->info.origin.orientation.w = 1.0;

    global_map_->data.assign(width_ * height_, 0);
}

void MapMemoryCore::updateMap(
    nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
    double robot_x,
    double robot_y,
    double robot_theta)
{
    const double local_res = local_costmap->info.resolution;
    const double local_origin_x = local_costmap->info.origin.position.x;
    const double local_origin_y = local_costmap->info.origin.position.y;
    const unsigned int local_w = local_costmap->info.width;
    const unsigned int local_h = local_costmap->info.height;

    const auto & local_data = local_costmap->data;

    const double cos_t = std::cos(robot_theta);
    const double sin_t = std::sin(robot_theta);

    for (unsigned int j = 0; j < local_h; ++j)
    {
        for (unsigned int i = 0; i < local_w; ++i)
        {
            const int8_t occ_val = local_data[j * local_w + i];

            if (occ_val < 0)
            {
                continue;
            }

            const double lx = local_origin_x + (static_cast<double>(i) + 0.5) * local_res;
            const double ly = local_origin_y + (static_cast<double>(j) + 0.5) * local_res;

            const double wx = robot_x + lx * cos_t - ly * sin_t;
            const double wy = robot_y + lx * sin_t + ly * cos_t;

            int gx = 0;
            int gy = 0;

            if (!worldToMap(wx, wy, gx, gy))
            {
                continue;
            }

            const int global_idx = index(gx, gy, static_cast<int>(global_map_->info.width));

            int8_t & global_val = global_map_->data[global_idx];

            const int current_global_cost = global_val < 0 ? 0 : static_cast<int>(global_val);
            const int local_cost = static_cast<int>(occ_val);

            global_val = static_cast<int8_t>(std::max(current_global_cost, local_cost));
        }
    }
}

bool MapMemoryCore::worldToMap(
    double world_x,
    double world_y,
    int & map_x,
    int & map_y) const
{
    if (world_x < origin_x_ || world_y < origin_y_)
    {
        return false;
    }

    map_x = static_cast<int>((world_x - origin_x_) / resolution_);
    map_y = static_cast<int>((world_y - origin_y_) / resolution_);

    if (map_x < 0 || map_y < 0 || map_x >= width_ || map_y >= height_)
    {
        return false;
    }

    return true;
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMapData() const
{
    return global_map_;
}

int MapMemoryCore::index(int x, int y, int width)
{
    return y * width + x;
}

} // namespace map_memory