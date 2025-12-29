#include "WaterWaypointNetwork.h"
#include "Map/TileManager.h"
#include "Map/SurfaceElement.h"
#include "RoutingMetrics.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <queue>

namespace OpenLoco::Vehicles
{
    static std::vector<Waypoint> _waypoints;
    static std::vector<WaterMassGroup> _waterMassGroups;
    static bool _initialized = false;
    static bool _dirty = true;

    namespace WaterWaypointNetwork
    {
        static void extractWaypoints()
        {
            _waypoints.clear();

            // Scan all tiles for water/land boundaries
            for (int16_t y = 0; y < World::kMapRows; ++y)
            {
                for (int16_t x = 0; x < World::kMapColumns; ++x)
                {
                    World::TilePos2 tilePos(x, y);
                    auto tile = World::TileManager::get(tilePos);
                    auto* surface = tile.surface();

                    if (surface == nullptr || surface->water() == 0)
                        continue;

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
                        // Try to offset 1 tile into open water if possible
                        World::TilePos2 waypointPos = tilePos;
                        
                        // Check all 4 directions for deeper water
                        static const World::TilePos2 kDirections[] = {
                            {0, -1}, {1, 0}, {0, 1}, {-1, 0}
                        };
                        
                        for (const auto& dir : kDirections)
                        {
                            World::TilePos2 offsetPos = tilePos + dir;
                            if (!World::validCoords(offsetPos))
                                continue;

                            auto offsetTile = World::TileManager::get(offsetPos);
                            auto* offsetSurface = offsetTile.surface();

                            if (offsetSurface != nullptr && offsetSurface->water() == surface->water())
                            {
                                // Check if this tile is surrounded by more water
                                bool moreOpen = true;
                                for (const auto& checkDir : kDirections)
                                {
                                    World::TilePos2 checkPos = offsetPos + checkDir;
                                    if (!World::validCoords(checkPos))
                                        continue;

                                    auto checkTile = World::TileManager::get(checkPos);
                                    auto* checkSurface = checkTile.surface();
                                    if (checkSurface == nullptr || checkSurface->water() == 0)
                                    {
                                        moreOpen = false;
                                        break;
                                    }
                                }

                                if (moreOpen)
                                {
                                    waypointPos = offsetPos;
                                    break;
                                }
                            }
                        }

                        Waypoint wp;
                        wp.pos = waypointPos;
                        wp.waterLevel = surface->water();
                        wp.groupId = 0xFFFF;
                        _waypoints.push_back(wp);
                    }
                }
            }
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

        static void buildConnections()
        {
            const int32_t kMaxConnectionDistance = 32;

            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
                auto& wp = _waypoints[i];
                wp.connections.clear();

                for (size_t j = 0; j < _waypoints.size(); ++j)
                {
                    if (i == j)
                        continue;

                    auto& other = _waypoints[j];
                    int32_t dx = std::abs(wp.pos.x - other.pos.x);
                    int32_t dy = std::abs(wp.pos.y - other.pos.y);
                    int32_t distance = dx + dy;

                    if (distance <= kMaxConnectionDistance && wp.waterLevel == other.waterLevel)
                    {
                        if (lineOfSightWater(wp.pos, other.pos, wp.waterLevel))
                        {
                            wp.connections.push_back(static_cast<uint16_t>(j));
                        }
                    }
                }
            }
        }

        static void buildWaterMassGroups()
        {
            _waterMassGroups.clear();

            if (_waypoints.empty())
                return;

            std::vector<bool> visited(_waypoints.size(), false);
            uint16_t currentGroupId = 0;

            for (size_t i = 0; i < _waypoints.size(); ++i)
            {
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
                    uint16_t current = toVisit.front();
                    toVisit.pop();

                    group.waypointIndices.push_back(current);
                    _waypoints[current].groupId = currentGroupId;

                    sumX += _waypoints[current].pos.x;
                    sumY += _waypoints[current].pos.y;
                    count++;

                    for (uint16_t neighborIdx : _waypoints[current].connections)
                    {
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
        }

        void initialize()
        {
            // Note: Waypoint network initialization doesn't use recursive pathfinding,
            // so no RIPF tracking needed here
            extractWaypoints();
            buildConnections();
            buildWaterMassGroups();

            Diagnostics::Logging::info("WaterWaypointNetwork: Initialized with {} waypoints, {} groups", 
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
