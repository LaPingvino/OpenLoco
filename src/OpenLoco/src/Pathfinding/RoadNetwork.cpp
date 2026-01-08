#include "RoadNetwork.h"
#include "Map/TileManager.h"
#include "Map/RoadElement.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Objects/ObjectManager.h"
#include "Objects/RoadObject.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

using namespace OpenLoco::World;
using namespace OpenLoco::World::Track;

namespace OpenLoco::Pathfinding
{
    void RoadNetwork::initialize()
    {
        Diagnostics::Logging::info("RoadNetwork: Initializing...");
        _nodes.clear();
        _posToNodeIndex.clear();
        _initialized = true;
        _dirty = true; // Will rebuild lazily
        _rebuildInProgress = false;
        _rebuildTileX = 0;
        _rebuildTileY = 0;
    }

    void RoadNetwork::reset()
    {
        Diagnostics::Logging::info("RoadNetwork: Resetting");
        _nodes.clear();
        _posToNodeIndex.clear();
        _initialized = false;
        _dirty = true;
        _ticksSinceRefresh = 0;
        _rebuildInProgress = false;
        _rebuildTileX = 0;
        _rebuildTileY = 0;
    }

    void RoadNetwork::markDirty()
    {
        _dirty = true;
    }

    void RoadNetwork::ensureInitialized()
    {
        if (!_initialized)
        {
            initialize();
        }
    }

    void RoadNetwork::updatePeriodicRefresh()
    {
        if (!_initialized)
            return;

        _ticksSinceRefresh++;

        // Continue incremental rebuild if in progress
        if (_dirty || _rebuildInProgress)
        {
            rebuildIncremental();
            return;
        }

        // Periodic full refresh
        if (_ticksSinceRefresh >= kRefreshInterval)
        {
            Diagnostics::Logging::info("RoadNetwork: Periodic refresh");
            markDirty();
            _ticksSinceRefresh = 0;
        }
    }

    uint64_t RoadNetwork::posToKey(Pos3 pos) const
    {
        return (static_cast<uint64_t>(pos.x) << 32) |
               (static_cast<uint64_t>(pos.y) << 16) |
               static_cast<uint64_t>(pos.z);
    }

    uint32_t RoadNetwork::getOrCreateNode(Pos3 pos, uint8_t roadObjectId, uint16_t rad, CompanyId owner, bool isTram)
    {
        uint64_t key = posToKey(pos);
        auto it = _posToNodeIndex.find(key);
        if (it != _posToNodeIndex.end())
        {
            return it->second;
        }

        uint32_t nodeIndex = static_cast<uint32_t>(_nodes.size());
        _nodes.push_back(RoadNode{
            pos,
            roadObjectId,
            rad,
            owner,
            isTram,
            false, // isOneWay - would need to check road object flags
            {},    // connections
            {},    // edgeCosts
            false  // isStation
        });
        _posToNodeIndex[key] = nodeIndex;
        return nodeIndex;
    }

    void RoadNetwork::addConnection(uint32_t fromNode, uint32_t toNode, uint16_t cost)
    {
        auto& node = _nodes[fromNode];

        // Check if connection already exists
        for (size_t i = 0; i < node.connections.size(); ++i)
        {
            if (node.connections[i] == toNode)
            {
                return; // Already connected
            }
        }

        if (node.connections.size() < 16)
        {
            node.connections.push_back(toNode);
            node.edgeCosts.push_back(cost);
        }
    }

    void RoadNetwork::rebuildIncremental()
    {
        if (!_rebuildInProgress)
        {
            // Start new rebuild
            _nodes.clear();
            _posToNodeIndex.clear();
            _rebuildTileX = 0;
            _rebuildTileY = 0;
            _rebuildInProgress = true;
            Diagnostics::Logging::info("RoadNetwork: Starting incremental rebuild");
        }

        // Process kTilesPerTick tiles
        uint32_t tilesProcessed = 0;
        while (tilesProcessed < kTilesPerTick && _rebuildTileY < kMapRows)
        {
            auto tile = TileManager::get(TilePos2(_rebuildTileX, _rebuildTileY));

            for (auto& el : tile)
            {
                auto* roadEl = el.as<RoadElement>();
                if (roadEl == nullptr)
                    continue;

                if (roadEl->isGhost() || roadEl->isAiAllocated())
                    continue;

                // Get road piece data
                auto roadId = roadEl->roadId();
                auto rotation = roadEl->rotation();
                auto roadObjectId = roadEl->roadObjectId();
                auto owner = roadEl->owner();
                auto baseZ = roadEl->baseHeight();

                // Check if this is a tram track
                auto* roadObj = ObjectManager::get<RoadObject>(roadObjectId);
                bool isTram = roadObj != nullptr && roadObj->hasFlags(RoadObjectFlags::isRail);

                // Create node for this road segment
                Pos3 roadPos = {
                    static_cast<coord_t>(_rebuildTileX * kTileSize),
                    static_cast<coord_t>(_rebuildTileY * kTileSize),
                    baseZ};

                uint16_t rad = (roadId << 3) | rotation;
                uint32_t nodeIdx = getOrCreateNode(roadPos, roadObjectId, rad, owner, isTram);

                // Get connections
                // Note: Simplified connection finding - in practice would use getRoadConnections()
                
                // Check adjacent tiles for connecting roads
                static const int16_t dx[] = {0, 1, 0, -1};
                static const int16_t dy[] = {-1, 0, 1, 0};

                for (int dir = 0; dir < 4; ++dir)
                {
                    int16_t nx = _rebuildTileX + dx[dir];
                    int16_t ny = _rebuildTileY + dy[dir];

                    if (nx < 0 || ny < 0 || nx >= kMapColumns || ny >= kMapRows)
                        continue;

                    auto neighborTile = TileManager::get(TilePos2(nx, ny));
                    for (auto& neighborEl : neighborTile)
                    {
                        auto* neighborRoad = neighborEl.as<RoadElement>();
                        if (neighborRoad == nullptr)
                            continue;

                        if (neighborRoad->isGhost() || neighborRoad->isAiAllocated())
                            continue;

                        // Check if same road type
                        if (neighborRoad->roadObjectId() != roadObjectId)
                            continue;

                        // Must be same or compatible owner
                        if (neighborRoad->owner() != owner && neighborRoad->owner() != CompanyId::neutral)
                            continue;

                        // Check if roads connect (simplified - check height compatibility)
                        if (std::abs(neighborRoad->baseHeight() - baseZ) > 16)
                            continue;

                        Pos3 neighborPos = {
                            static_cast<coord_t>(nx * kTileSize),
                            static_cast<coord_t>(ny * kTileSize),
                            neighborRoad->baseHeight()};

                        uint16_t neighborRad = (neighborRoad->roadId() << 3) | neighborRoad->rotation();
                        uint32_t neighborNodeIdx = getOrCreateNode(neighborPos, neighborRoad->roadObjectId(), neighborRad, neighborRoad->owner(), isTram);

                        // Cost based on road length
                        uint16_t cost = 10; // Base cost per segment

                        addConnection(nodeIdx, neighborNodeIdx, cost);
                    }
                }
            }

            tilesProcessed++;
            _rebuildTileX++;
            if (_rebuildTileX >= kMapColumns)
            {
                _rebuildTileX = 0;
                _rebuildTileY++;
            }
        }

        // Check if rebuild is complete
        if (_rebuildTileY >= kMapRows)
        {
            _rebuildInProgress = false;
            _dirty = false;
            Diagnostics::Logging::info("RoadNetwork: Rebuild complete, {} nodes", _nodes.size());
        }
    }

    void RoadNetwork::buildFromMap()
    {
        // Force immediate full rebuild
        _dirty = true;
        _rebuildInProgress = false;

        while (_dirty || _rebuildInProgress)
        {
            rebuildIncremental();
        }
    }

    std::optional<uint32_t> RoadNetwork::findNearestNode(Pos3 pos, uint8_t roadObjectId, bool isTram) const
    {
        std::optional<uint32_t> nearest;
        uint32_t nearestDist = UINT32_MAX;

        for (uint32_t i = 0; i < _nodes.size(); ++i)
        {
            const auto& node = _nodes[i];
            if (node.roadObjectId != roadObjectId)
                continue;
            if (node.isTramTrack != isTram)
                continue;

            int32_t dx = node.pos.x - pos.x;
            int32_t dy = node.pos.y - pos.y;
            int32_t dz = node.pos.z - pos.z;
            uint32_t dist = std::abs(dx) + std::abs(dy) + std::abs(dz);

            if (dist < nearestDist)
            {
                nearestDist = dist;
                nearest = i;
            }
        }

        return nearest;
    }

    bool RoadNetwork::isConnected(uint32_t fromNode, uint32_t toNode) const
    {
        if (fromNode >= _nodes.size())
            return false;

        const auto& node = _nodes[fromNode];
        for (auto conn : node.connections)
        {
            if (conn == toNode)
                return true;
        }
        return false;
    }

    uint32_t RoadNetwork::heuristic(uint32_t nodeA, uint32_t nodeB) const
    {
        const auto& a = _nodes[nodeA];
        const auto& b = _nodes[nodeB];

        // Manhattan distance
        return std::abs(a.pos.x - b.pos.x) +
               std::abs(a.pos.y - b.pos.y) +
               std::abs(a.pos.z - b.pos.z);
    }

    RoadPathResult RoadNetwork::astarSearch(uint32_t startNode, uint32_t goalNode, CompanyId company, bool isTram)
    {
        RoadPathResult result;
        result.hasPath = false;

        if (startNode >= _nodes.size() || goalNode >= _nodes.size())
            return result;

        struct AStarNode
        {
            uint32_t nodeIdx;
            uint32_t gScore; // Cost from start
            uint32_t fScore; // gScore + heuristic

            bool operator>(const AStarNode& other) const
            {
                return fScore > other.fScore;
            }
        };

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
        std::unordered_map<uint32_t, uint32_t> gScores;
        std::unordered_map<uint32_t, uint32_t> cameFrom;
        std::unordered_set<uint32_t> closedSet;

        gScores[startNode] = 0;
        openSet.push({startNode, 0, heuristic(startNode, goalNode)});

        constexpr size_t kMaxIterations = 50000;
        size_t iterations = 0;

        while (!openSet.empty() && iterations < kMaxIterations)
        {
            iterations++;

            AStarNode current = openSet.top();
            openSet.pop();

            if (current.nodeIdx == goalNode)
            {
                // Reconstruct path
                result.hasPath = true;
                result.totalCost = current.gScore;

                uint32_t node = goalNode;
                while (node != startNode)
                {
                    result.nodeIndices.push_back(node);
                    result.positions.push_back(_nodes[node].pos);

                    auto it = cameFrom.find(node);
                    if (it == cameFrom.end())
                        break;
                    node = it->second;
                }
                result.nodeIndices.push_back(startNode);
                result.positions.push_back(_nodes[startNode].pos);

                std::reverse(result.nodeIndices.begin(), result.nodeIndices.end());
                std::reverse(result.positions.begin(), result.positions.end());

                return result;
            }

            if (closedSet.count(current.nodeIdx) > 0)
                continue;

            closedSet.insert(current.nodeIdx);

            const auto& node = _nodes[current.nodeIdx];

            for (size_t i = 0; i < node.connections.size(); ++i)
            {
                uint32_t neighbor = node.connections[i];
                uint16_t edgeCost = node.edgeCosts[i];

                if (closedSet.count(neighbor) > 0)
                    continue;

                // Check compatibility
                const auto& neighborNode = _nodes[neighbor];

                // Must match tram/road type
                if (neighborNode.isTramTrack != isTram)
                    continue;

                // Check ownership compatibility
                if (neighborNode.owner != company && neighborNode.owner != CompanyId::neutral)
                {
                    // Can't use road owned by another company (unless neutral)
                    continue;
                }

                uint32_t tentativeG = current.gScore + edgeCost;

                auto gIt = gScores.find(neighbor);
                if (gIt != gScores.end() && tentativeG >= gIt->second)
                    continue;

                gScores[neighbor] = tentativeG;
                cameFrom[neighbor] = current.nodeIdx;

                uint32_t fScore = tentativeG + heuristic(neighbor, goalNode);
                openSet.push({neighbor, tentativeG, fScore});
            }
        }

        Diagnostics::Logging::warn("RoadNetwork: A* failed after {} iterations", iterations);
        return result;
    }

    RoadPathResult RoadNetwork::findPath(Pos3 from, Pos3 to, uint8_t roadObjectId, CompanyId company, bool isTram)
    {
        ensureInitialized();

        // Wait for rebuild to complete if in progress
        if (_rebuildInProgress)
        {
            return RoadPathResult{};
        }

        auto startNode = findNearestNode(from, roadObjectId, isTram);
        auto goalNode = findNearestNode(to, roadObjectId, isTram);

        if (!startNode.has_value() || !goalNode.has_value())
        {
            Diagnostics::Logging::warn("RoadNetwork: Could not find start or goal node");
            return RoadPathResult{};
        }

        return astarSearch(*startNode, *goalNode, company, isTram);
    }
}
