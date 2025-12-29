#include "WaterWaypointPathfinding.h"
#include "WaterWaypointNetwork.h"
#include "VehicleHead.h"
#include "RoutingMetrics.h"
#include "Orders.h"
#include "World/StationManager.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <cmath>

namespace OpenLoco::Vehicles
{
    namespace WaterWaypointPathfinding
    {
        struct AStarNode
        {
            uint16_t waypointIdx;
            uint16_t gCost;
            uint16_t hCost;
            uint16_t fCost() const { return gCost + hCost; }
            uint16_t parent;

            bool operator>(const AStarNode& other) const
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

        WaypointPathResult waypointBasedPathfind(const VehicleHead& head, World::TilePos2 targetPos, World::MicroZ waterLevel)
        {
            uint32_t astarIterations = 0;

            WaypointPathResult result;
            result.hasPath = false;
            result.direction = 0xFF;
            result.pathCost = 0xFFFF;
            result.startWaypoint = 0xFFFF;
            result.targetWaypoint = 0xFFFF;

            WaterWaypointNetwork::ensureInitialized();

            World::TilePos2 currentPos(head.position.x / World::kTileSize, head.position.y / World::kTileSize);

            uint16_t startWaypointIdx = WaterWaypointNetwork::findNearestWaypoint(currentPos, waterLevel);
            uint16_t targetWaypointIdx = WaterWaypointNetwork::findNearestWaypoint(targetPos, waterLevel);

            if (startWaypointIdx == 0xFFFF || targetWaypointIdx == 0xFFFF)
            {
                Diagnostics::Logging::verbose("WaypointPathfinding: No waypoints found (start={}, target={})", 
                    startWaypointIdx, targetWaypointIdx);
                return result;
            }

            auto* startWaypoint = WaterWaypointNetwork::getWaypoint(startWaypointIdx);
            auto* targetWaypoint = WaterWaypointNetwork::getWaypoint(targetWaypointIdx);

            if (startWaypoint == nullptr || targetWaypoint == nullptr)
                return result;

            // Check if both waypoints are in the same water mass group
            if (startWaypoint->groupId != targetWaypoint->groupId)
            {
                Diagnostics::Logging::verbose("WaypointPathfinding: Different water groups (start group={}, target group={})", 
                    startWaypoint->groupId, targetWaypoint->groupId);
                return result;
            }

            Diagnostics::Logging::verbose("WaypointPathfinding: A* from waypoint {} to {} (groups match: {})", 
                startWaypointIdx, targetWaypointIdx, startWaypoint->groupId);

            result.startWaypoint = startWaypointIdx;
            result.targetWaypoint = targetWaypointIdx;

            // A* pathfinding
            std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
            std::unordered_map<uint16_t, uint16_t> gScores;
            std::unordered_map<uint16_t, uint16_t> cameFrom;

            AStarNode startNode;
            startNode.waypointIdx = startWaypointIdx;
            startNode.gCost = 0;
            startNode.hCost = manhattanDistance(startWaypoint->pos, targetWaypoint->pos);
            startNode.parent = 0xFFFF;

            openSet.push(startNode);
            gScores[startWaypointIdx] = 0;

            while (!openSet.empty())
            {
                astarIterations++;
                AStarNode current = openSet.top();
                openSet.pop();

                if (current.waypointIdx == targetWaypointIdx)
                {
                    // Reconstruct path
                    std::vector<uint16_t> waypointPath;
                    uint16_t pathIdx = targetWaypointIdx;

                    while (pathIdx != 0xFFFF)
                    {
                        waypointPath.push_back(pathIdx);
                        auto it = cameFrom.find(pathIdx);
                        if (it == cameFrom.end())
                            break;
                        pathIdx = it->second;
                    }

                    std::reverse(waypointPath.begin(), waypointPath.end());

                    // Convert waypoint path to tile positions
                    for (uint16_t wpIdx : waypointPath)
                    {
                        auto* wp = WaterWaypointNetwork::getWaypoint(wpIdx);
                        if (wp != nullptr)
                        {
                            result.routePoints.push_back(wp->pos);
                        }
                    }

                    // Calculate direction to first waypoint
                    if (!result.routePoints.empty())
                    {
                        auto& firstWaypoint = result.routePoints[0];
                        int32_t dx = firstWaypoint.x - currentPos.x;
                        int32_t dy = firstWaypoint.y - currentPos.y;

                        if (std::abs(dx) > std::abs(dy))
                            result.direction = dx > 0 ? 1 : 3;
                        else
                            result.direction = dy > 0 ? 2 : 0;
                    }

                    result.hasPath = true;
                    result.pathCost = current.gCost;
                    result.startWaypoint = startWaypointIdx;
                    result.targetWaypoint = targetWaypointIdx;
                    
                    Diagnostics::Logging::verbose("WaypointPathfinding: Path found! {} waypoints, direction={}, cost={}", 
                        result.routePoints.size(), result.direction, result.pathCost);
                    
                    // Record RIPF for waypoint pathfinding
                    RoutingMetrics::recordWaterPathfindCall(astarIterations);
                    return result;
                }

                auto* currentWaypoint = WaterWaypointNetwork::getWaypoint(current.waypointIdx);
                if (currentWaypoint == nullptr)
                    continue;

                for (uint16_t neighborIdx : currentWaypoint->connections)
                {
                    auto* neighbor = WaterWaypointNetwork::getWaypoint(neighborIdx);
                    if (neighbor == nullptr)
                        continue;

                    // Only follow connections within same group
                    if (neighbor->groupId != startWaypoint->groupId)
                        continue;

                    uint16_t tentativeGScore = current.gCost + manhattanDistance(currentWaypoint->pos, neighbor->pos);

                    auto it = gScores.find(neighborIdx);
                    if (it == gScores.end() || tentativeGScore < it->second)
                    {
                        gScores[neighborIdx] = tentativeGScore;
                        cameFrom[neighborIdx] = current.waypointIdx;

                        AStarNode neighborNode;
                        neighborNode.waypointIdx = neighborIdx;
                        neighborNode.gCost = tentativeGScore;
                        neighborNode.hCost = manhattanDistance(neighbor->pos, targetWaypoint->pos);
                        neighborNode.parent = current.waypointIdx;

                        openSet.push(neighborNode);
                    }
                }
            }

            // Record RIPF for waypoint pathfinding (no path found case)
            RoutingMetrics::recordWaterPathfindCall(astarIterations);
            return result;
        }
    }
}
