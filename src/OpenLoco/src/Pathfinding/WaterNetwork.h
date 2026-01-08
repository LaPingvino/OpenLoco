#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenLoco::Pathfinding
{
    // Path result from water pathfinding
    struct WaterPathResult
    {
        bool hasPath;
        std::vector<World::TilePos2> waypoints;  // Simplified waypoints (every N tiles)
        std::vector<World::TilePos2> fullPath;   // Complete tile-by-tile path for debug
    };

    // Single water level bitmap with quadrant cache
    struct WaterBitmap
    {
        std::vector<std::vector<uint64_t>> bits;
        std::vector<std::vector<std::vector<uint32_t>>> quadrantCache;
        uint32_t paddedSize = 0;
        uint32_t numLevels = 0;
        bool dirty = true;

        void build(World::MicroZ waterLevel);
        void buildQuadrantCache();
        bool isWater(World::TilePos2 pos) const;
        uint32_t getQuadrantWaterCount(uint32_t level, uint32_t qx, uint32_t qy) const;
    };

    // Height-indexed water network supporting multiple water levels
    class WaterNetwork
    {
    public:
        // Lifecycle
        void initialize();
        void reset();
        void markDirty();
        void ensureInitialized();
        void updatePeriodicRefresh();

        // Single tile water query - O(1) bit lookup
        bool isWater(World::TilePos2 pos, World::MicroZ waterLevel);

        // Quadrant queries for a specific water level
        uint32_t getQuadrantWaterCount(World::MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy);
        bool isPureWater(World::MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy);
        bool isAllLand(World::MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy);

        // Main pathfinding entry point
        WaterPathResult findPath(World::TilePos2 from, World::TilePos2 to, World::MicroZ waterLevel);

        // Height-aware queries
        std::vector<World::MicroZ> getWaterLevels() const;
        bool hasWaterLevel(World::MicroZ level) const;

        // Debug/info
        uint32_t getPaddedSize() const;
        uint32_t getNumLevels() const;
        uint32_t getNodeCount() const;

    private:
        // Height-indexed water bitmaps (key = water MicroZ level)
        // Most maps have just 1 water level, but this supports multiple
        std::unordered_map<World::MicroZ, WaterBitmap> _waterLevels;

        bool _initialized = false;
        bool _dirty = true;

        // Periodic refresh tracking
        uint32_t _ticksSinceRefresh = 0;
        uint32_t _ticksAtGoodFps = 0;

        // Lazy rebuild - only rebuild a portion per tick
        uint32_t _rebuildRowIndex = 0;
        bool _rebuildInProgress = false;

        static constexpr uint32_t kTicksPerMinute = 60 * 60;
        static constexpr uint32_t kFastRefreshInterval = kTicksPerMinute;
        static constexpr uint32_t kSlowRefreshInterval = kTicksPerMinute * 10;
        static constexpr uint32_t kRowsPerTick = 16; // Lazy rebuild: process N rows per tick

        void scanWaterLevels();
        void rebuildIncremental();
        WaterBitmap& getBitmapForLevel(World::MicroZ level);
        bool isTileWater(int32_t tx, int32_t ty, World::MicroZ waterLevel) const;
    };
}
