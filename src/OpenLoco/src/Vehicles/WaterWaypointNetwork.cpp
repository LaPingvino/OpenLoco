#include "WaterWaypointNetwork.h"
#include "Map/TileManager.h"
#include "Map/SurfaceElement.h"
#include "RoutingMetrics.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <queue>
#include <unordered_map>

namespace OpenLoco::Vehicles
{
    static std::vector<Waypoint> _waypoints;
    static std::vector<WaterMassGroup> _waterMassGroups;
    static bool _initialized = false;
    static bool _dirty = true;
    static std::unordered_map<uint32_t, std::vector<uint16_t>> _spatialGrid;
    static const int32_t kMaxConnectionDistance = 32;
    static const int32_t kCellSize = kMaxConnectionDistance;

    namespace WaterWaypointNetwork
    {
        static uint32_t getCellKey(World::TilePos2 pos)
        {
            int32_t cellX = pos.x / kCellSize;
            int32_t cellY = pos.y / kCellSize;
            return (static_cast<uint32_t>(cellX) << 16) | static_cast<uint32_t>(cellY);
        }
        
        static void extractWaypoints()
        {
            _waypoints.clear();
            uint32_t operationCount = 0;

            // Scan all tiles for water/land boundaries
            for (int16_t y = 0; y < World::kMapRows; ++y)
            {
                for (int16_t x = 0; x < World::kMapColumns; ++x)
                {
                    operationCount++; // Count each tile check
                    World::TilePos2 tilePos(x, y);
                    auto tile = World::TileManager::get(tilePos);
                    auto* surface = tile.surface();

                    if (surface == nullptr || surface->water() == 0)
                        continue;

                    operationCount++; // Water tile found, checking neighbors
                    // This is a water tile - check if it's near land
                    bool nearLand = false;
                    for (int8_t dy = -1; dy <= 1 && !nearLand; ++dy)
                    {
                        for (int8_t dx = -1; dx <= 1; ++dx)
                        {
                            if (dx == 0 && dy == 0)
                                continue;

                            World::TilePos2 neighborPos(x + dx, y + dy);
                            if (!World::validCoords(neighborPos))
                                continue;

                            auto neighborTile = World::TileManager::get(neighborPos);
                            auto* neighborSurface = neighborTile.surface();

                            if (neighborSurface == nullptr || neighborSurface->water() == 0)
                            {
                                nearLand = true;
                                break;
                            }
                        }
                    }

                    if (nearLand)
                    {
                        World::TilePos2 waypointPos = tilePos;
                        
                        // Check all 4 directions for deeper water
                        static const World::TilePos2 kDirections[] = {
                            {0, -1}, {1, 0}, {0, 1}, {-1, 0}
                        };
                        
                        // First, try to find a spot with 4 tiles clearance and 2-tile margin
                        bool foundGoodSpot = false;
                        for (const auto& dir : kDirections)
                        {
                            for (int32_t offset = 4; offset >= 1 && !foundGoodSpot; --offset)
                            {
                                World::TilePos2 offsetPos = tilePos + (dir * offset);
                                if (!World::validCoords(offsetPos))
                                    continue;

                                auto offsetTile = World::TileManager::get(offsetPos);
                                auto* offsetSurface = offsetTile.surface();

                                if (offsetSurface == nullptr || offsetSurface->water() != surface->water())
                                    continue;

                                // Check if this tile has good water clearance (at least 2 tiles in all directions)
                                bool hasGoodClearance = true;
                                for (int8_t checkDy = -2; checkDy <= 2 && hasGoodClearance; ++checkDy)
                                {
                                    for (int8_t checkDx = -2; checkDx <= 2; ++checkDx)
                                    {
                                        if (checkDx == 0 && checkDy == 0)
                                            continue;

                                        World::TilePos2 checkPos = offsetPos + World::TilePos2{checkDx, checkDy};
                                        if (!World::validCoords(checkPos))
                                            continue;

                                        auto checkTile = World::TileManager::get(checkPos);
                                        auto* checkSurface = checkTile.surface();
                                        if (checkSurface == nullptr || checkSurface->water() != surface->water())
                                        {
                                            hasGoodClearance = false;
                                            break;
                                        }
                                    }
                                }

                                if (hasGoodClearance)
                                {
                                    waypointPos = offsetPos;
                                    foundGoodSpot = true;
                                    break;
                                }
                            }
                            
                            if (foundGoodSpot)
                                break;
                        }
                        
                        // If we couldn't find ideal clearance, try to center in the waterway
                        if (!foundGoodSpot)
                        {
                            // For each direction, measure how far we can go before hitting land
                            int32_t distances[4] = {0, 0, 0, 0};
                            
                            for (int dir = 0; dir < 4; ++dir)
                            {
                                for (int32_t dist = 1; dist <= 10; ++dist)
                                {
                                    World::TilePos2 checkPos = tilePos + (kDirections[dir] * dist);
                                    if (!World::validCoords(checkPos))
                                        break;
                                    
                                    auto checkTile = World::TileManager::get(checkPos);
                                    auto* checkSurface = checkTile.surface();
                                    
                                    if (checkSurface == nullptr || checkSurface->water() != surface->water())
                                        break;
                                    
                                    distances[dir] = dist;
                                }
                            }
                            
                            // Find the direction with most water and move halfway in that direction
                            int32_t bestDist = 0;
                            int bestDir = -1;
                            for (int dir = 0; dir < 4; ++dir)
                            {
                                if (distances[dir] > bestDist)
                                {
                                    bestDist = distances[dir];
                                    bestDir = dir;
                                }
                            }
                            
                            // Move to center of the waterway (halfway to the farthest point)
                            if (bestDir >= 0 && bestDist > 1)
                            {
                                int32_t centerOffset = bestDist / 2;
                                if (centerOffset > 0)
                                {
                                    World::TilePos2 centerPos = tilePos + (kDirections[bestDir] * centerOffset);
                                    if (World::validCoords(centerPos))
                                    {
                                        auto centerTile = World::TileManager::get(centerPos);
                                        auto* centerSurface = centerTile.surface();
                                        if (centerSurface != nullptr && centerSurface->water() == surface->water())
                                        {
                                            waypointPos = centerPos;
                                        }
                                    }
                                }
                            }
                        }

                        operationCount++; // Waypoint placement computation
                        Waypoint wp;
                        wp.pos = waypointPos;
                        wp.waterLevel = surface->water();
                        wp.groupId = 0xFFFF;
                        _waypoints.push_back(wp);
                    }
                }
            }
            
            // Record the cost of scanning the entire map for waypoints
            RoutingMetrics::recordWaterPathfindCall(operationCount);
        }

        static bool lineOfSightWater(World::TilePos2 from, World::TilePos2 to, World::MicroZ waterLevel)
        {
            int32_t dx = std::abs(to.x - from.x);
            int32_t dy = std::abs(to.y - from.y);
            int32_t sx = from.x < to.x ? 1 : -1;
            int32_t sy = from.y < to.y ? 1 : -1;
            int32_t err = dx - dy;

            World::TilePos2 current = from;

            while (current.x != to.x || current.y != to.y)
            {
                if (!World::validCoords(current))
                    return false;

                auto tile = World::TileManager::get(current);
                auto* surface = tile.surface();

                if (surface == nullptr || surface->water() != waterLevel)
                    return false;

                int32_t e2 = 2 * err;
                if (e2 > -dy)
                {
                    err -= dy;
                    current.x += sx;
                }
                if (e2 < dx)
                {
                    err += dx;
                    current.y += sy;
                }
            }

            return true;
        }
        
        // Tile-by-tile A* pathfinding to check if two waypoints are connected through narrow channels
        static bool tileAStarConnectable(World::TilePos2 from, World::TilePos2 to, World::MicroZ waterLevel, uint32_t& iterations)
        {
            iterations = 0;
            
            // Maximum search depth to prevent excessive computation
            const uint32_t kMaxIterations = 200;
            
            struct AStarNode
            {
                World::TilePos2 pos;
                uint16_t gCost;
                uint16_t hCost;
                uint16_t fCost() const { return gCost + hCost; }
                
                bool operator>(const AStarNode& other) const
                {
                    return fCost() > other.fCost();
                }
            };
            
            auto manhattanDistance = [](World::TilePos2 a, World::TilePos2 b) -> uint16_t {
                int32_t dx = std::abs(a.x - b.x);
                int32_t dy = std::abs(a.y - b.y);
                return static_cast<uint16_t>(dx + dy);
            };
            
            std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
            std::unordered_map<uint32_t, uint16_t> gScores;
            
            auto encodePos = [](World::TilePos2 pos) -> uint32_t {
                return (static_cast<uint32_t>(pos.x) << 16) | static_cast<uint32_t>(pos.y);
            };
            
            AStarNode startNode;
            startNode.pos = from;
            startNode.gCost = 0;
            startNode.hCost = manhattanDistance(from, to);
            
            openSet.push(startNode);
            gScores[encodePos(from)] = 0;
            
            static const World::TilePos2 kDirections[] = {
                {0, -1}, {1, 0}, {0, 1}, {-1, 0}
            };
            
            while (!openSet.empty() && iterations < kMaxIterations)
            {
                iterations++;
                AStarNode current = openSet.top();
                openSet.pop();
                
                // Reached target?
                if (current.pos.x == to.x && current.pos.y == to.y)
                    return true;
                
                // Try all 4 directions
                for (const auto& dir : kDirections)
                {
                    World::TilePos2 neighbor = current.pos + dir;
                    
                    if (!World::validCoords(neighbor))
                        continue;
                    
                    auto tile = World::TileManager::get(neighbor);
                    auto* surface = tile.surface();
                    
                    if (surface == nullptr || surface->water() != waterLevel)
                        continue;
                    
                    uint16_t tentativeGCost = current.gCost + 1;
                    uint32_t neighborKey = encodePos(neighbor);
                    
                    auto it = gScores.find(neighborKey);
                    if (it == gScores.end() || tentativeGCost < it->second)
                    {
                        gScores[neighborKey] = tentativeGCost;
                        
                        AStarNode neighborNode;
                        neighborNode.pos = neighbor;
                        neighborNode.gCost = tentativeGCost;
                        neighborNode.hCost = manhattanDistance(neighbor, to);
                        
                        openSet.push(neighborNode);
                    }
                }
            }
            
            return false;
        }

        // Build spatial grid for fast neighbor lookups (called once during initialization)
        static void buildSpatialGrid()
        {
            _spatialGrid.clear();
            
            // Populate spatial grid
            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
                uint32_t cellKey = getCellKey(_waypoints[i].pos);
                _spatialGrid[cellKey].push_back(static_cast<uint16_t>(i));
            }
        }
        
        // Phase 1: Build connections using only line-of-sight (fast)
        static void buildLineOfSightConnections()
        {
            uint32_t operationCount = 0;
            uint32_t lineOfSightChecks = 0;
            
            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
                auto& wp = _waypoints[i];
                wp.connections.clear();

                int32_t cellX = wp.pos.x / kCellSize;
                int32_t cellY = wp.pos.y / kCellSize;
                
                // Check 3x3 grid of cells around this waypoint
                for (int32_t dy = -1; dy <= 1; ++dy)
                {
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        uint32_t cellKey = (static_cast<uint32_t>(cellX + dx) << 16) | static_cast<uint32_t>(cellY + dy);
                        auto it = _spatialGrid.find(cellKey);
                        if (it == _spatialGrid.end())
                            continue;
                        
                        for (uint16_t j : it->second)
                        {
                            if (i == j)
                                continue;

                            operationCount++; // Count waypoint pair check
                            auto& other = _waypoints[j];
                            int32_t distX = std::abs(wp.pos.x - other.pos.x);
                            int32_t distY = std::abs(wp.pos.y - other.pos.y);
                            int32_t distance = distX + distY;

                            if (distance <= kMaxConnectionDistance && wp.waterLevel == other.waterLevel)
                            {
                                // Only line-of-sight in phase 1 (fast)
                                lineOfSightChecks++;
                                if (lineOfSightWater(wp.pos, other.pos, wp.waterLevel))
                                {
                                    wp.connections.push_back(static_cast<uint16_t>(j));
                                }
                            }
                        }
                    }
                }
            }
            
            // Record cost of line-of-sight connection building
            uint32_t lineOfSightCost = lineOfSightChecks * 16; // Average ~16 tiles per check
            RoutingMetrics::recordWaterPathfindCall(operationCount + lineOfSightCost);
        }
        
        // Phase 2: Try to connect nearby groups using tile A* for narrow channels
        static void connectNearbyGroups()
        {
            uint32_t totalAStarIterations = 0;
            
            // For each pair of adjacent groups, try to find a connecting channel
            for (size_t groupA = 0; groupA < _waterMassGroups.size(); ++groupA)
            {
                for (size_t groupB = groupA + 1; groupB < _waterMassGroups.size(); ++groupB)
                {
                    auto& groupAData = _waterMassGroups[groupA];
                    auto& groupBData = _waterMassGroups[groupB];
                    
                    // Check if group centers are close enough to potentially connect
                    int32_t dx = std::abs(groupAData.centerPoint.x - groupBData.centerPoint.x) / World::kTileSize;
                    int32_t dy = std::abs(groupAData.centerPoint.y - groupBData.centerPoint.y) / World::kTileSize;
                    int32_t centerDistance = dx + dy;
                    
                    // Only try if groups are reasonably close (within 100 tiles)
                    if (centerDistance > 100)
                        continue;
                    
                    // Find closest waypoint pair between the two groups
                    uint16_t bestWpA = 0xFFFF;
                    uint16_t bestWpB = 0xFFFF;
                    int32_t bestDistance = std::numeric_limits<int32_t>::max();
                    
                    for (uint16_t wpAIdx : groupAData.waypointIndices)
                    {
                        for (uint16_t wpBIdx : groupBData.waypointIndices)
                        {
                            auto& wpA = _waypoints[wpAIdx];
                            auto& wpB = _waypoints[wpBIdx];
                            
                            int32_t dist = std::abs(wpA.pos.x - wpB.pos.x) + std::abs(wpA.pos.y - wpB.pos.y);
                            if (dist < bestDistance && wpA.waterLevel == wpB.waterLevel)
                            {
                                bestDistance = dist;
                                bestWpA = wpAIdx;
                                bestWpB = wpBIdx;
                            }
                        }
                    }
                    
                    // Try tile A* between closest waypoints if they're close enough
                    if (bestWpA != 0xFFFF && bestWpB != 0xFFFF && bestDistance <= kMaxConnectionDistance * 2)
                    {
                        auto& wpA = _waypoints[bestWpA];
                        auto& wpB = _waypoints[bestWpB];
                        
                        uint32_t iterations = 0;
                        if (tileAStarConnectable(wpA.pos, wpB.pos, wpA.waterLevel, iterations))
                        {
                            // Found a connection! Add bidirectional edges
                            wpA.connections.push_back(bestWpB);
                            wpB.connections.push_back(bestWpA);
                            totalAStarIterations += iterations;
                            
                            Diagnostics::Logging::verbose("WaterWaypointNetwork: Connected groups {} and {} via narrow channel (distance={})", 
                                groupA, groupB, bestDistance);
                        }
                        else
                        {
                            totalAStarIterations += iterations;
                        }
                    }
                }
            }
            
            // Record tile A* cost
            if (totalAStarIterations > 0)
            {
                RoutingMetrics::recordWaterPathfindCall(totalAStarIterations);
            }
        }

        static void buildWaterMassGroups()
        {
            _waterMassGroups.clear();

            if (_waypoints.empty())
                return;

            uint32_t operationCount = 0;
            std::vector<bool> visited(_waypoints.size(), false);
            uint16_t currentGroupId = 0;

            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
                operationCount++; // Check each waypoint
                if (visited[i])
                    continue;

                WaterMassGroup group;
                group.groupId = currentGroupId;

                std::queue<uint16_t> toVisit;
                toVisit.push(static_cast<uint16_t>(i));
                visited[i] = true;

                int64_t sumX = 0, sumY = 0;
                uint32_t count = 0;

                while (!toVisit.empty())
                {
                    operationCount++; // BFS iteration
                    uint16_t current = toVisit.front();
                    toVisit.pop();

                    group.waypointIndices.push_back(current);
                    _waypoints[current].groupId = currentGroupId;

                    sumX += _waypoints[current].pos.x;
                    sumY += _waypoints[current].pos.y;
                    count++;

                    for (uint16_t neighborIdx : _waypoints[current].connections)
                    {
                        operationCount++; // Check each connection
                        if (!visited[neighborIdx])
                        {
                            visited[neighborIdx] = true;
                            toVisit.push(neighborIdx);
                        }
                    }
                }

                if (count > 0)
                {
                    group.centerPoint = World::Pos2(
                        static_cast<int16_t>((sumX / count) * World::kTileSize),
                        static_cast<int16_t>((sumY / count) * World::kTileSize)
                    );
                }

                _waterMassGroups.push_back(group);
                currentGroupId++;
            }
            
            // Record the cost of grouping waypoints via BFS
            RoutingMetrics::recordWaterPathfindCall(operationCount);
        }

        void initialize()
        {
            // Three-phase initialization for waypoint network:
            
            // Phase 1: Extract waypoints from map (tracks tile scanning)
            extractWaypoints();
            
            // Build spatial index for fast neighbor lookups
            buildSpatialGrid();
            
            // Phase 2: Build connections using only fast line-of-sight checks
            buildLineOfSightConnections();
            
            // Phase 3: Initial grouping based on line-of-sight connections
            buildWaterMassGroups();
            
            Diagnostics::Logging::info("WaterWaypointNetwork: Initial grouping - {} waypoints, {} groups", 
                _waypoints.size(), _waterMassGroups.size());
            
            // Phase 4: Try to connect nearby groups using tile A* for narrow channels
            connectNearbyGroups();
            
            // Phase 5: Re-run grouping to merge newly connected groups
            buildWaterMassGroups();

            Diagnostics::Logging::info("WaterWaypointNetwork: Final grouping - {} waypoints, {} groups", 
                _waypoints.size(), _waterMassGroups.size());

            _initialized = true;
            _dirty = false;
        }

        void ensureInitialized()
        {
            if (!_initialized || _dirty)
            {
                initialize();
            }
        }

        void markDirty()
        {
            _dirty = true;
        }

        uint16_t findNearestWaypoint(World::TilePos2 pos, World::MicroZ waterLevel)
        {
            ensureInitialized();

            if (_waypoints.empty())
                return 0xFFFF;

            uint16_t bestIdx = 0xFFFF;
            int32_t bestDistance = std::numeric_limits<int32_t>::max();

            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
                if (_waypoints[i].waterLevel != waterLevel)
                    continue;

                int32_t dx = std::abs(_waypoints[i].pos.x - pos.x);
                int32_t dy = std::abs(_waypoints[i].pos.y - pos.y);
                int32_t distance = dx + dy;

                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    bestIdx = static_cast<uint16_t>(i);
                }
            }

            return bestIdx;
        }

        const Waypoint* getWaypoint(uint16_t index)
        {
            ensureInitialized();

            if (index >= _waypoints.size())
                return nullptr;

            return &_waypoints[index];
        }

        const std::vector<Waypoint>& getAllWaypoints()
        {
            ensureInitialized();
            return _waypoints;
        }

        const std::vector<WaterMassGroup>& getWaterMassGroups()
        {
            ensureInitialized();
            return _waterMassGroups;
        }
    }
}
