#pragma once

#include "World/CompanyManager.h"
#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>
#include <sfl/static_vector.hpp>

namespace OpenLoco::Pathfinding
{
    // Node represents a track segment endpoint or junction
    struct TrackNode
    {
        World::Pos3 pos;
        uint8_t trackObjectId;
        uint16_t trackAndDirection; // TrackAndDirection packed value
        CompanyId owner;

        // Connections to other nodes (max 16 per junction)
        sfl::static_vector<uint32_t, 16> connections; // Node indices
        sfl::static_vector<uint16_t, 16> edgeCosts;   // Cost to reach each connection

        bool hasSignal = false;
        bool isStation = false;
    };

    // Result from track pathfinding
    struct TrackPathResult
    {
        bool hasPath = false;
        std::vector<uint32_t> nodeIndices;
        std::vector<World::Pos3> positions;
        uint32_t totalCost = 0;
    };

    // Cached path for a vehicle
    struct CachedTrackPath
    {
        World::Pos3 targetPos;
        std::vector<uint32_t> nodeIndices;
        std::vector<World::Pos3> waypoints;
        size_t currentWaypointIndex = 0;
        bool isValid = false;
        uint8_t failureCount = 0;
        static constexpr uint8_t kMaxFailures = 10;
    };

    // Graph structure for rail network pathfinding
    class TrackNetwork
    {
    public:
        // Lifecycle
        void initialize();
        void reset();
        void markDirty();
        void ensureInitialized();
        void updatePeriodicRefresh();

        // Build network from map track elements
        void buildFromMap();

        // A* pathfinding
        TrackPathResult findPath(
            World::Pos3 from,
            World::Pos3 to,
            uint8_t trackObjectId,
            CompanyId company);

        // Find nearest node to a position
        std::optional<uint32_t> findNearestNode(World::Pos3 pos, uint8_t trackObjectId) const;

        // Network queries
        uint32_t getNodeCount() const { return static_cast<uint32_t>(_nodes.size()); }
        const TrackNode& getNode(uint32_t index) const { return _nodes[index]; }
        bool isConnected(uint32_t fromNode, uint32_t toNode) const;

        // Debug visualization
        const std::vector<TrackNode>& getNodes() const { return _nodes; }

    private:
        std::vector<TrackNode> _nodes;
        std::unordered_map<uint64_t, uint32_t> _posToNodeIndex; // Fast lookup by position

        bool _initialized = false;
        bool _dirty = true;

        // Periodic refresh tracking
        uint32_t _ticksSinceRefresh = 0;
        static constexpr uint32_t kTicksPerMinute = 60 * 60;
        static constexpr uint32_t kRefreshInterval = kTicksPerMinute * 5; // 5 minutes

        // Lazy rebuild state
        bool _rebuildInProgress = false;
        int16_t _rebuildTileX = 0;
        int16_t _rebuildTileY = 0;
        static constexpr uint32_t kTilesPerTick = 64; // Process N tiles per tick

        // Helper functions
        uint64_t posToKey(World::Pos3 pos) const;
        uint32_t getOrCreateNode(World::Pos3 pos, uint8_t trackObjectId, uint16_t tad, CompanyId owner);
        void addConnection(uint32_t fromNode, uint32_t toNode, uint16_t cost);
        void rebuildIncremental();

        // A* implementation
        TrackPathResult astarSearch(uint32_t startNode, uint32_t goalNode, CompanyId company);
        uint32_t heuristic(uint32_t nodeA, uint32_t nodeB) const;
    };
}
