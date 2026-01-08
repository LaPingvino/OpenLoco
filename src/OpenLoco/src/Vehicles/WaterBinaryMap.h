#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <vector>
#include <cstdint>

namespace OpenLoco::Vehicles::WaterBinaryMap
{
    // Path result from hierarchical pathfinding
    struct PathResult
    {
        bool hasPath;
        std::vector<World::TilePos2> waypoints;   // Simplified waypoints (every N tiles)
        std::vector<World::TilePos2> fullPath;    // Complete tile-by-tile path for debug
    };

    // Lifecycle management
    void initialize();
    void ensureInitialized();
    void markDirty();
    void reset();
    
    // Periodic refresh tracking (call every game tick)
    void updatePeriodicRefresh();

    // Single tile water query - O(1) bit lookup
    bool isWater(World::TilePos2 pos, World::MicroZ waterLevel);

    // Quadrant queries
    uint32_t getQuadrantWaterCount(uint32_t level, uint32_t qx, uint32_t qy);
    bool isPureWater(uint32_t level, uint32_t qx, uint32_t qy);
    bool isAllLand(uint32_t level, uint32_t qx, uint32_t qy);

    // Main pathfinding entry point
    PathResult findPath(World::TilePos2 from, World::TilePos2 to, World::MicroZ waterLevel);

    // Debug/info
    uint32_t getPaddedSize();
    uint32_t getNumLevels();
}
