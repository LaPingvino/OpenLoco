#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <vector>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    struct VehicleHead;

    struct RegionPathResult
    {
        bool hasPath;
        uint8_t direction;
        uint16_t pathCost;
        uint16_t startRegion;
        uint16_t targetRegion;
        std::vector<World::TilePos2> routePoints; // Complete tile-by-tile path
        std::vector<uint16_t> channelIds; // Channels to traverse in order
    };

    namespace WaterWaypointPathfinding
    {
        // Region-based pathfinding: greedy in regions, follow channels between them
        RegionPathResult regionBasedPathfind(const VehicleHead& head, World::TilePos2 targetPos, World::MicroZ waterLevel);
    }
}
