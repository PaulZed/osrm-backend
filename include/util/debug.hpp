#ifndef OSRM_ENGINE_GUIDANCE_DEBUG_HPP_
#define OSRM_ENGINE_GUIDANCE_DEBUG_HPP_

#include "engine/guidance/route_step.hpp"

#include <iostream>
#include <vector>

namespace osrm
{
namespace util
{
inline void print(const RouteStep &step)
{
    std::cout << static_cast<int>(step.maneuver.instruction.type) << " "
              << static_cast<int>(step.maneuver.instruction.direction_modifier) << "  "
              << static_cast<int>(step.maneuver.waypoint_type) << " Duration: " << step.duration
              << " Distance: " << step.distance << " Geometry: " << step.geometry_begin << " "
              << step.geometry_end << " exit: " << step.maneuver.exit
              << " Intersections: " << step.intersections.size() << " [";

    for (const auto &intersection : step.intersections)
    {
        std::cout << "(bearings:";
        for (auto bearing : intersection.bearings)
            std::cout << " " << bearing;
        std::cout << ", entry: ";
        for (auto entry : intersection.entry)
            std::cout << " " << entry;
        std::cout << ")";
    }
    std::cout << "] name[" << step.name_id << "]: " << step.name;
}

inline void print(const std::vector<RouteStep> &steps)
{
    std::cout << "Path\n";
    int segment = 0;
    for (const auto &step : steps)
    {
        std::cout << "\t[" << segment++ << "]: ";
        print(step);
        std::cout << std::endl;
    }
}

inline void print(const LaneDataVector &turn_lane_data)
{
    std::cout << " Tags:\n";
    for (auto entry : turn_lane_data)
        std::cout << "\t" << entry.tag << " from: " << static_cast<int>(entry.from)
                  << " to: " << static_cast<int>(entry.to) << "\n";
    std::cout << std::flush;
}

inline void printTurnAssignmentData(const NodeID at,
                                    const LaneDataVector &turn_lane_data,
                                    const Intersection &intersection,
                                    const std::vector<QueryNode> &node_info_list)
{
    std::cout << "[Turn Assignment Progress]\nLocation:";
    auto coordinate = node_info_list[at];
    std::cout << std::setprecision(12) << toFloating(coordinate.lat) << " "
              << toFloating(coordinate.lon) << "\n";

    std::cout << "  Intersection:\n";
    for (const auto &road)
        std::cout << "\t" << toString(road) << "\n";

    // flushes as well
    print(turn_lane_data);
}

} // namespace util
} // namespace osrm

#endif /*OSRM_ENGINE_GUIDANCE_DEBUG_HPP_*/
