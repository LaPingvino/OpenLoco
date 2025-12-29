#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <vector>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    struct Waypoint
    {
        World::TilePos2 pos;
        World::MicroZ waterLevel;
        uint16_t groupId;
        std::vector<uint16_t> connections;
    };

    struct WaterMassGroup
    {
        uint16_t groupId;
        std::vector<uint16_t> waypointIndices;
        World::Pos2 centerPoint;
    };

    namespace WaterWaypointNetwork
    {
        void initialize();
        void ensureInitialized();
        void markDirty();
        
        uint16_t findNearestWaypoint(World::TilePos2 pos, World::MicroZ waterLevel);
        const Waypoint* getWaypoint(uint16_t index);
        bool tryConnectWaypoints(uint16_t waypointA, uint16_t waypointB); // On-demand connection via tile A*
        const std::vector<Waypoint>& getAllWaypoints();
        const std::vector<WaterMassGroup>& getWaterMassGroups();
    }
}
