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
    // Node represents a road segment endpoint or junction
    struct RoadNode
    {
        World::Pos3 pos;
        uint8_t roadObjectId;
        uint16_t roadAndDirection; // RoadAndDirection packed value
        CompanyId owner;

        bool isTramTrack = false;  // Differentiate tram vs road
        bool isOneWay = false;

        // Connections to other nodes (max 16 per junction)
        sfl::static_vector<uint32_t, 16> connections; // Node indices
        sfl::static_vector<uint16_t, 16> edgeCosts;   // Cost to reach each connection

        bool isStation = false;
    };

    // Result from road pathfinding
    struct RoadPathResult
    {
        bool hasPath = false;
        std::vector<uint32_t> nodeIndices;
        std::vector<World::Pos3> positions;
        uint32_t totalCost = 0;
    };

    // Cached path for a road vehicle
    struct CachedRoadPath
    {
        World::Pos3 targetPos;
        std::vector<uint32_t> nodeIndices;
        std::vector<World::Pos3> waypoints;
        size_t currentWaypointIndex = 0;
        bool isValid = false;
        uint8_t failureCount = 0;
        static constexpr uint8_t kMaxFailures = 10;
    };

    // Graph structure for road network pathfinding
    class RoadNetwork
    {
    public:
        // Lifecycle
        void initialize();
        void reset();
        void markDirty();
        void ensureInitialized();
        void updatePeriodicRefresh();

        // Build network from map road elements
        void buildFromMap();

        // A* pathfinding - separate for trams vs buses/trucks
        RoadPathResult findPath(
            World::Pos3 from,
            World::Pos3 to,
            uint8_t roadObjectId,
            CompanyId company,
            bool isTram);

        // Find nearest node to a position
        std::optional<uint32_t> findNearestNode(World::Pos3 pos, uint8_t roadObjectId, bool isTram) const;

        // Network queries
        uint32_t getNodeCount() const { return static_cast<uint32_t>(_nodes.size()); }
        const RoadNode& getNode(uint32_t index) const { return _nodes[index]; }
        bool isConnected(uint32_t fromNode, uint32_t toNode) const;

        // Debug visualization
        const std::vector<RoadNode>& getNodes() const { return _nodes; }

    private:
        std::vector<RoadNode> _nodes;
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
        uint32_t getOrCreateNode(World::Pos3 pos, uint8_t roadObjectId, uint16_t rad, CompanyId owner, bool isTram);
        void addConnection(uint32_t fromNode, uint32_t toNode, uint16_t cost);
        void rebuildIncremental();

        // A* implementation
        RoadPathResult astarSearch(uint32_t startNode, uint32_t goalNode, CompanyId company, bool isTram);
        uint32_t heuristic(uint32_t nodeA, uint32_t nodeB) const;
    };
}
