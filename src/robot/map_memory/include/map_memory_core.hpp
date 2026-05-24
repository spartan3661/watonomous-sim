#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace map_memory
{

class MapMemoryCore
{
public:
    MapMemoryCore(
        double resolution,
        int width,
        int height,
        double origin_x,
        double origin_y,
        const std::string & frame_id);

    void updateMap(
        nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
        double robot_x,
        double robot_y,
        double robot_theta);

    nav_msgs::msg::OccupancyGrid::SharedPtr getMapData() const;

private:
    void initializeGlobalMap();

    bool worldToMap(
        double world_x,
        double world_y,
        int & map_x,
        int & map_y) const;

    static int index(int x, int y, int width);

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;

    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::string frame_id_;
};

} // namespace map_memory

#endif