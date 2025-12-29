#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <vector>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;

    struct WaypointPathResult
    {
        bool hasPath;
        uint8_t direction;
        uint16_t pathCost;
        uint16_t startWaypoint;
        uint16_t targetWaypoint;
        std::vector<World::TilePos2> routePoints;
        std::vector<uint16_t> waypointIndices; // Waypoint indices corresponding to routePoints
    };

    namespace WaterWaypointPathfinding
    {
        WaypointPathResult waypointBasedPathfind(const VehicleHead& head, World::TilePos2 targetPos, World::MicroZ waterLevel);
    }
}
