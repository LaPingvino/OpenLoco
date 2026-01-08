#include "TrackNetwork.h"
#include "Map/TileManager.h"
#include "Map/TrackElement.h"
#include "Map/Track/Track.h"
#include "Map/Track/TrackData.h"
#include "Objects/ObjectManager.h"
#include "Objects/TrackObject.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

using namespace OpenLoco::World;
using namespace OpenLoco::World::Track;

namespace OpenLoco::Pathfinding
{
    void TrackNetwork::initialize()
    {
        Diagnostics::Logging::info("TrackNetwork: Initializing...");
        _nodes.clear();
        _posToNodeIndex.clear();
        _initialized = true;
        _dirty = true; // Will rebuild lazily
        _rebuildInProgress = false;
        _rebuildTileX = 0;
        _rebuildTileY = 0;
    }

    void TrackNetwork::reset()
    {
        Diagnostics::Logging::info("TrackNetwork: Resetting");
        _nodes.clear();
        _posToNodeIndex.clear();
        _initialized = false;
        _dirty = true;
        _ticksSinceRefresh = 0;
        _rebuildInProgress = false;
        _rebuildTileX = 0;
        _rebuildTileY = 0;
    }

    void TrackNetwork::markDirty()
    {
        _dirty = true;
    }

    void TrackNetwork::ensureInitialized()
    {
        if (!_initialized)
        {
            initialize();
        }
    }

    void TrackNetwork::updatePeriodicRefresh()
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
            Diagnostics::Logging::info("TrackNetwork: Periodic refresh");
            markDirty();
            _ticksSinceRefresh = 0;
        }
    }

    uint64_t TrackNetwork::posToKey(Pos3 pos) const
    {
        // Pack position into 64-bit key
        return (static_cast<uint64_t>(pos.x) << 32) |
               (static_cast<uint64_t>(pos.y) << 16) |
               static_cast<uint64_t>(pos.z);
    }

    uint32_t TrackNetwork::getOrCreateNode(Pos3 pos, uint8_t trackObjectId, uint16_t tad, CompanyId owner)
    {
        uint64_t key = posToKey(pos);
        auto it = _posToNodeIndex.find(key);
        if (it != _posToNodeIndex.end())
        {
            return it->second;
        }

        uint32_t nodeIndex = static_cast<uint32_t>(_nodes.size());
        _nodes.push_back(TrackNode{
            pos,
            trackObjectId,
            tad,
            owner,
            {}, // connections
            {}, // edgeCosts
            false,
            false});
        _posToNodeIndex[key] = nodeIndex;
        return nodeIndex;
    }

    void TrackNetwork::addConnection(uint32_t fromNode, uint32_t toNode, uint16_t cost)
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

    void TrackNetwork::rebuildIncremental()
    {
        if (!_rebuildInProgress)
        {
            // Start new rebuild
            _nodes.clear();
            _posToNodeIndex.clear();
            _rebuildTileX = 0;
            _rebuildTileY = 0;
            _rebuildInProgress = true;
            Diagnostics::Logging::info("TrackNetwork: Starting incremental rebuild");
        }

        // Process kTilesPerTick tiles
        uint32_t tilesProcessed = 0;
        while (tilesProcessed < kTilesPerTick && _rebuildTileY < kMapRows)
        {
            auto tile = TileManager::get(TilePos2(_rebuildTileX, _rebuildTileY));

            for (auto& el : tile)
            {
                auto* trackEl = el.as<TrackElement>();
                if (trackEl == nullptr)
                    continue;

                if (trackEl->isGhost() || trackEl->isAiAllocated())
                    continue;

                // Get track piece data
                auto trackId = trackEl->trackId();
                auto rotation = trackEl->rotation();
                auto trackObjectId = trackEl->trackObjectId();
                auto owner = trackEl->owner();
                auto baseZ = trackEl->baseHeight();

                // Create node for this track segment
                Pos3 trackPos = {
                    static_cast<coord_t>(_rebuildTileX * kTileSize),
                    static_cast<coord_t>(_rebuildTileY * kTileSize),
                    baseZ};

                uint16_t tad = (trackId << 3) | rotation;
                uint32_t nodeIdx = getOrCreateNode(trackPos, trackObjectId, tad, owner);

                // Mark if has signal
                if (trackEl->hasSignal())
                {
                    _nodes[nodeIdx].hasSignal = true;
                }

                // Get connections using existing game logic
                TrackConnections connections = {};
                Pos3 connectionPos = trackPos;

                // Get track piece info for connection finding
                auto& trackPieces = TrackData::getTrackPiece(trackId);
                if (!trackPieces.empty())
                {
                    // Calculate connection point at end of track piece
                    auto& lastPiece = trackPieces.back();
                    auto rotatedOffset = Math::Vector::rotate(
                        Pos2{lastPiece.x, lastPiece.y}, rotation);
                    connectionPos.x += rotatedOffset.x;
                    connectionPos.y += rotatedOffset.y;
                    connectionPos.z += lastPiece.z;
                }

                // Find connected track elements
                auto connectionTile = TileManager::get(connectionPos);
                for (auto& connEl : connectionTile)
                {
                    auto* connTrackEl = connEl.as<TrackElement>();
                    if (connTrackEl == nullptr)
                        continue;

                    if (connTrackEl->isGhost() || connTrackEl->isAiAllocated())
                        continue;

                    // Must be same track type or compatible
                    if (connTrackEl->trackObjectId() != trackObjectId)
                        continue;

                    // Must be same or compatible owner
                    if (connTrackEl->owner() != owner && connTrackEl->owner() != CompanyId::neutral)
                        continue;

                    Pos3 connPos = {
                        connectionPos.x,
                        connectionPos.y,
                        connTrackEl->baseHeight()};

                    uint16_t connTad = (connTrackEl->trackId() << 3) | connTrackEl->rotation();
                    uint32_t connNodeIdx = getOrCreateNode(connPos, connTrackEl->trackObjectId(), connTad, connTrackEl->owner());

                    // Cost based on track length (simplified)
                    uint16_t cost = 10; // Base cost per segment

                    addConnection(nodeIdx, connNodeIdx, cost);
                    addConnection(connNodeIdx, nodeIdx, cost); // Bidirectional
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
            Diagnostics::Logging::info("TrackNetwork: Rebuild complete, {} nodes", _nodes.size());
        }
    }

    void TrackNetwork::buildFromMap()
    {
        // Force immediate full rebuild
        _dirty = true;
        _rebuildInProgress = false;

        while (_dirty || _rebuildInProgress)
        {
            rebuildIncremental();
        }
    }

    std::optional<uint32_t> TrackNetwork::findNearestNode(Pos3 pos, uint8_t trackObjectId) const
    {
        std::optional<uint32_t> nearest;
        uint32_t nearestDist = UINT32_MAX;

        for (uint32_t i = 0; i < _nodes.size(); ++i)
        {
            const auto& node = _nodes[i];
            if (node.trackObjectId != trackObjectId)
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

    bool TrackNetwork::isConnected(uint32_t fromNode, uint32_t toNode) const
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

    uint32_t TrackNetwork::heuristic(uint32_t nodeA, uint32_t nodeB) const
    {
        const auto& a = _nodes[nodeA];
        const auto& b = _nodes[nodeB];

        // Manhattan distance
        return std::abs(a.pos.x - b.pos.x) +
               std::abs(a.pos.y - b.pos.y) +
               std::abs(a.pos.z - b.pos.z);
    }

    TrackPathResult TrackNetwork::astarSearch(uint32_t startNode, uint32_t goalNode, CompanyId company)
    {
        TrackPathResult result;
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

                // Check ownership compatibility
                const auto& neighborNode = _nodes[neighbor];
                if (neighborNode.owner != company && neighborNode.owner != CompanyId::neutral)
                {
                    // Can't use track owned by another company
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

        Diagnostics::Logging::warn("TrackNetwork: A* failed after {} iterations", iterations);
        return result;
    }

    TrackPathResult TrackNetwork::findPath(Pos3 from, Pos3 to, uint8_t trackObjectId, CompanyId company)
    {
        ensureInitialized();

        // Wait for rebuild to complete if in progress
        if (_rebuildInProgress)
        {
            return TrackPathResult{};
        }

        auto startNode = findNearestNode(from, trackObjectId);
        auto goalNode = findNearestNode(to, trackObjectId);

        if (!startNode.has_value() || !goalNode.has_value())
        {
            Diagnostics::Logging::warn("TrackNetwork: Could not find start or goal node");
            return TrackPathResult{};
        }

        return astarSearch(*startNode, *goalNode, company);
    }
}
