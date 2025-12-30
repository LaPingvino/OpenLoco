#include "WaterWaypointPathfinding.h"
#include "WaterWaypointNetwork.h"
#include "VehicleHead.h"
#include "RoutingMetrics.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace OpenLoco::Vehicles
{
    namespace WaterWaypointPathfinding
    {
        struct RegionAStarNode
        {
            uint16_t regionId;
            uint16_t gCost;
            uint16_t hCost;
            uint16_t fCost() const { return gCost + hCost; }

            bool operator>(const RegionAStarNode& other) const
            {
                return fCost() > other.fCost();
            }
        };

        static uint16_t manhattanDistance(World::TilePos2 a, World::TilePos2 b)
        {
            int32_t dx = std::abs(a.x - b.x);
            int32_t dy = std::abs(a.y - b.y);
            return static_cast<uint16_t>(dx + dy);
        }

        RegionPathResult regionBasedPathfind(const VehicleHead& head, World::TilePos2 targetPos, World::MicroZ waterLevel)
        {
            uint32_t astarIterations = 0;

            RegionPathResult result;
            result.hasPath = false;
            result.direction = 0xFF;
            result.pathCost = 0xFFFF;
            result.startRegion = 0xFFFF;
            result.targetRegion = 0xFFFF;

            WaterWaypointNetwork::ensureInitialized();

            World::TilePos2 currentPos(head.position.x / World::kTileSize, head.position.y / World::kTileSize);

            // Find which regions we're in
            uint16_t startRegion = WaterWaypointNetwork::findRegionAt(currentPos, waterLevel);
            uint16_t targetRegion = WaterWaypointNetwork::findRegionAt(targetPos, waterLevel);

            if (startRegion == 0xFFFF || targetRegion == 0xFFFF)
            {
                Diagnostics::Logging::verbose("WaterWaypointPathfinding: Ship or target not in a water region (start={}, target={})",
                    startRegion, targetRegion);
                return result;
            }

            result.startRegion = startRegion;
            result.targetRegion = targetRegion;

            auto* startRegionPtr = WaterWaypointNetwork::getRegion(startRegion);
            auto* targetRegionPtr = WaterWaypointNetwork::getRegion(targetRegion);

            if (startRegionPtr == nullptr || targetRegionPtr == nullptr)
                return result;

            Diagnostics::Logging::verbose("WaterWaypointPathfinding: Region A* from region {} to {} (water level {})",
                startRegion, targetRegion, waterLevel);

            // If in same region, use greedy movement
            if (startRegion == targetRegion)
            {
                Diagnostics::Logging::verbose("WaterWaypointPathfinding: Same region - greedy movement");
                
                // Simple greedy path (just the target for now)
                result.routePoints.push_back(targetPos);
                result.hasPath = true;
                result.pathCost = manhattanDistance(currentPos, targetPos);

                // Calculate direction
                int32_t dx = targetPos.x - currentPos.x;
                int32_t dy = targetPos.y - currentPos.y;

                if (std::abs(dx) > std::abs(dy))
                    result.direction = dx > 0 ? 1 : 3;
                else
                    result.direction = dy > 0 ? 2 : 0;

                return result;
            }

            // Different regions - use A* to find region path
            std::priority_queue<RegionAStarNode, std::vector<RegionAStarNode>, std::greater<RegionAStarNode>> openSet;
            std::unordered_map<uint16_t, uint16_t> gScores;
            std::unordered_map<uint16_t, uint16_t> cameFrom;
            std::unordered_map<uint16_t, uint16_t> channelUsed; // Track which channel was used to reach each region

            RegionAStarNode startNode;
            startNode.regionId = startRegion;
            startNode.gCost = 0;
            startNode.hCost = manhattanDistance(startRegionPtr->centerPoint, targetRegionPtr->centerPoint);

            openSet.push(startNode);
            gScores[startRegion] = 0;

            while (!openSet.empty())
            {
                astarIterations++;
                RegionAStarNode current = openSet.top();
                openSet.pop();

                if (current.regionId == targetRegion)
                {
                    // Reconstruct region path
                    std::vector<uint16_t> regionPath;
                    std::vector<uint16_t> channelPath;
                    uint16_t pathRegion = targetRegion;

                    while (pathRegion != startRegion)
                    {
                        regionPath.push_back(pathRegion);
                        
                        auto channelIt = channelUsed.find(pathRegion);
                        if (channelIt != channelUsed.end())
                        {
                            channelPath.push_back(channelIt->second);
                        }
                        
                        auto it = cameFrom.find(pathRegion);
                        if (it == cameFrom.end())
                            break;
                        pathRegion = it->second;
                    }

                    regionPath.push_back(startRegion);
                    
                    std::reverse(regionPath.begin(), regionPath.end());
                    std::reverse(channelPath.begin(), channelPath.end());

                    result.channelIds = channelPath;

                    // Build tile path: use region centers as waypoints and channels when available
                    for (size_t i = 1; i < regionPath.size(); ++i) // Start from 1 to skip starting region
                    {
                        // Check if there's a channel for this transition
                        if (i - 1 < channelPath.size())
                        {
                            auto* channel = WaterWaypointNetwork::getChannel(channelPath[i - 1]);
                            if (channel != nullptr)
                            {
                                // Add channel tiles to route
                                for (const auto& tile : channel->path)
                                {
                                    result.routePoints.push_back(tile);
                                }
                                continue; // Skip adding region center since we have the channel path
                            }
                        }
                        
                        // No channel - use region center as waypoint
                        auto* regionPtr = WaterWaypointNetwork::getRegion(regionPath[i]);
                        if (regionPtr != nullptr)
                        {
                            result.routePoints.push_back(regionPtr->centerPoint);
                        }
                    }

                    // Add final destination
                    result.routePoints.push_back(targetPos);

                    result.hasPath = true;
                    result.pathCost = static_cast<uint16_t>(result.routePoints.size());

                    // Calculate direction to first waypoint
                    if (!result.routePoints.empty())
                    {
                        auto& firstPoint = result.routePoints[0];
                        int32_t dx = firstPoint.x - currentPos.x;
                        int32_t dy = firstPoint.y - currentPos.y;

                        if (std::abs(dx) > std::abs(dy))
                            result.direction = dx > 0 ? 1 : 3;
                        else
                            result.direction = dy > 0 ? 2 : 0;
                    }

                    Diagnostics::Logging::verbose("WaterWaypointPathfinding: Path found! {} regions, {} channels, {} tiles",
                        regionPath.size(), channelPath.size(), result.routePoints.size());

                    RoutingMetrics::recordWaterPathfindCall(astarIterations);
                    return result;
                }

                auto* currentRegionPtr = WaterWaypointNetwork::getRegion(current.regionId);
                if (currentRegionPtr == nullptr)
                    continue;

                // Explore connected regions (via adjacency or channels)
                for (size_t i = 0; i < currentRegionPtr->connectedRegions.size(); ++i)
                {
                    uint16_t neighborRegion = currentRegionPtr->connectedRegions[i];
                    
                    auto* neighborRegionPtr = WaterWaypointNetwork::getRegion(neighborRegion);
                    if (neighborRegionPtr == nullptr)
                        continue;

                    // Calculate cost (use channel if available, otherwise use distance between centers)
                    uint16_t movementCost;
                    uint16_t channelId = 0xFFFF;
                    
                    if (i < currentRegionPtr->connectedChannels.size())
                    {
                        channelId = currentRegionPtr->connectedChannels[i];
                        auto* channel = WaterWaypointNetwork::getChannel(channelId);
                        if (channel != nullptr)
                        {
                            movementCost = static_cast<uint16_t>(channel->path.size());
                        }
                        else
                        {
                            // Adjacent region without channel - use direct distance
                            movementCost = manhattanDistance(currentRegionPtr->centerPoint, neighborRegionPtr->centerPoint);
                            channelId = 0xFFFF;
                        }
                    }
                    else
                    {
                        // Adjacent region without channel - use direct distance
                        movementCost = manhattanDistance(currentRegionPtr->centerPoint, neighborRegionPtr->centerPoint);
                    }

                    uint16_t tentativeGScore = current.gCost + movementCost;

                    auto it = gScores.find(neighborRegion);
                    if (it == gScores.end() || tentativeGScore < it->second)
                    {
                        gScores[neighborRegion] = tentativeGScore;
                        cameFrom[neighborRegion] = current.regionId;
                        if (channelId != 0xFFFF)
                        {
                            channelUsed[neighborRegion] = channelId;
                        }

                        RegionAStarNode neighborNode;
                        neighborNode.regionId = neighborRegion;
                        neighborNode.gCost = tentativeGScore;
                        neighborNode.hCost = manhattanDistance(neighborRegionPtr->centerPoint, targetRegionPtr->centerPoint);

                        openSet.push(neighborNode);
                    }
                }
            }

            Diagnostics::Logging::info("WaterWaypointPathfinding: No path found after {} iterations", astarIterations);
            RoutingMetrics::recordWaterPathfindCall(astarIterations);
            return result;
        }
    }
}
