#include "WaterWaypointNetwork.h"
#include "Map/TileManager.h"
#include "Map/SurfaceElement.h"
#include "Map/StationElement.h"
#include "World/StationManager.h"
#include "RoutingMetrics.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using namespace OpenLoco::World;

namespace OpenLoco::Vehicles
{
    static std::vector<WaterRegion> _regions;
    static std::vector<WaterChannel> _channels;
    static bool _initialized = false;
    static bool _dirty = true;
    
    // Map from tile position to region ID (for fast lookup)
    static std::unordered_map<uint32_t, uint16_t> _tileToRegion;

    namespace WaterWaypointNetwork
    {
        static uint32_t getTileKey(World::TilePos2 pos)
        {
            return (static_cast<uint32_t>(pos.x) << 16) | static_cast<uint32_t>(pos.y);
        }
        
        static bool isWaterTile(World::TilePos2 pos, World::MicroZ waterLevel)
        {
            if (!World::validCoords(pos))
                return false;
            
            auto tile = World::TileManager::get(pos);
            auto* surface = tile.surface();
            
            return surface != nullptr && surface->water() == waterLevel;
        }
        
        // Count water neighbors
        static int countWaterNeighbors(World::TilePos2 pos, World::MicroZ waterLevel)
        {
            static const World::TilePos2 kDirections[] = {
                {0, -1}, {1, 0}, {0, 1}, {-1, 0}
            };
            
            int count = 0;
            for (const auto& dir : kDirections)
            {
                if (isWaterTile(pos + dir, waterLevel))
                    count++;
            }
            return count;
        }
        
        // Classify tiles as open water or narrow channel
        static bool isNarrowChannel(World::TilePos2 pos, World::MicroZ waterLevel)
        {
            int waterNeighbors = countWaterNeighbors(pos, waterLevel);
            
            // Narrow channel: exactly 2 opposite water neighbors (forms a line)
            // or 2 adjacent neighbors (forms a bend)
            if (waterNeighbors != 2)
                return false;
            
            // If it has exactly 2 water neighbors, it's probably a channel
            // unless it's part of a wider area
            return true;
        }
        
        // Flood-fill to find a contiguous water region
        // Subdivides large regions to ensure convexity for greedy navigation
        static void floodFillRegion(World::TilePos2 start, World::MicroZ waterLevel,
                                     std::unordered_set<uint32_t>& visited)
        {
            const int32_t kMaxRegionSize = 16; // Max dimension for a region to ensure convexity
            const int32_t kMinRegionTiles = 10; // Minimum tiles for a region (smaller = treat as channel)
            
            std::queue<World::TilePos2> toVisit;
            toVisit.push(start);
            visited.insert(getTileKey(start));
            
            std::vector<World::TilePos2> allTiles;
            
            static const World::TilePos2 kDirections[] = {
                {0, -1}, {1, 0}, {0, 1}, {-1, 0}
            };
            
            // First, collect all tiles in this water mass
            while (!toVisit.empty())
            {
                World::TilePos2 current = toVisit.front();
                toVisit.pop();
                
                allTiles.push_back(current);
                
                // Explore neighbors (but skip narrow channels - they'll be handled separately)
                for (const auto& dir : kDirections)
                {
                    World::TilePos2 neighbor = current + dir;
                    uint32_t neighborKey = getTileKey(neighbor);
                    
                    if (visited.count(neighborKey) > 0)
                        continue;
                    
                    if (!isWaterTile(neighbor, waterLevel))
                        continue;
                    
                    // Skip if it's a narrow channel (we'll handle these separately)
                    if (isNarrowChannel(neighbor, waterLevel))
                        continue;
                    
                    visited.insert(neighborKey);
                    toVisit.push(neighbor);
                }
            }
            
            // Subdivide into smaller regions based on spatial proximity
            // Use a grid-based approach: split into kMaxRegionSize x kMaxRegionSize cells
            if (allTiles.empty())
                return;
            
            // Find bounds
            int16_t minX = allTiles[0].x, maxX = allTiles[0].x;
            int16_t minY = allTiles[0].y, maxY = allTiles[0].y;
            
            for (const auto& tile : allTiles)
            {
                minX = std::min(minX, tile.x);
                maxX = std::max(maxX, tile.x);
                minY = std::min(minY, tile.y);
                maxY = std::max(maxY, tile.y);
            }
            
            // Create grid of subregions
            std::unordered_map<uint32_t, std::vector<World::TilePos2>> cellTiles;
            
            for (const auto& tile : allTiles)
            {
                int32_t cellX = (tile.x - minX) / kMaxRegionSize;
                int32_t cellY = (tile.y - minY) / kMaxRegionSize;
                uint32_t cellKey = (static_cast<uint32_t>(cellX) << 16) | static_cast<uint32_t>(cellY);
                cellTiles[cellKey].push_back(tile);
            }
            
            // Create a region for each non-empty cell, but only if it's pure water
            for (const auto& [cellKey, tiles] : cellTiles)
            {
                if (tiles.empty())
                    continue;
                
                // Check if this cell is pure water (no land tiles)
                // Calculate the bounding box of tiles in this cell
                int16_t cellMinX = tiles[0].x;
                int16_t cellMaxX = tiles[0].x;
                int16_t cellMinY = tiles[0].y;
                int16_t cellMaxY = tiles[0].y;
                
                for (const auto& tile : tiles)
                {
                    cellMinX = std::min(cellMinX, tile.x);
                    cellMaxX = std::max(cellMaxX, tile.x);
                    cellMinY = std::min(cellMinY, tile.y);
                    cellMaxY = std::max(cellMaxY, tile.y);
                }
                
                // Count how many tiles should be in a fully filled rectangle
                int32_t expectedTiles = (cellMaxX - cellMinX + 1) * (cellMaxY - cellMinY + 1);
                int32_t actualTiles = tiles.size();
                
                // If the cell contains land (actual < expected), we need to split it further
                // Use flood fill to create separate regions for each contiguous water area
                if (actualTiles < expectedTiles)
                {
                    // Cell contains land - split it into multiple regions via flood fill
                    std::unordered_set<uint32_t> cellVisited;
                    
                    for (const auto& tile : tiles)
                    {
                        if (cellVisited.count(getTileKey(tile)) > 0)
                            continue;
                        
                        // Flood fill from this tile to find a contiguous sub-region
                        std::queue<World::TilePos2> subQueue;
                        std::vector<World::TilePos2> subRegionTiles;
                        
                        subQueue.push(tile);
                        cellVisited.insert(getTileKey(tile));
                        
                        while (!subQueue.empty())
                        {
                            World::TilePos2 current = subQueue.front();
                            subQueue.pop();
                            subRegionTiles.push_back(current);
                            
                            static const World::TilePos2 kDirections[] = {
                                {0, -1}, {1, 0}, {0, 1}, {-1, 0}
                            };
                            
                            for (const auto& dir : kDirections)
                            {
                                World::TilePos2 neighbor = current + dir;
                                uint32_t neighborKey = getTileKey(neighbor);
                                
                                if (cellVisited.count(neighborKey) > 0)
                                    continue;
                                
                                // Check if neighbor is in our tiles list
                                bool found = false;
                                for (const auto& t : tiles)
                                {
                                    if (t.x == neighbor.x && t.y == neighbor.y)
                                    {
                                        found = true;
                                        break;
                                    }
                                }
                                
                                if (found)
                                {
                                    cellVisited.insert(neighborKey);
                                    subQueue.push(neighbor);
                                }
                            }
                        }
                        
                        // Create a region for this sub-region (only if large enough)
                        if (!subRegionTiles.empty() && static_cast<int32_t>(subRegionTiles.size()) >= kMinRegionTiles)
                        {
                            WaterRegion region;
                            region.waterLevel = waterLevel;
                            region.tiles = subRegionTiles;
                            
                            region.minX = subRegionTiles[0].x;
                            region.maxX = subRegionTiles[0].x;
                            region.minY = subRegionTiles[0].y;
                            region.maxY = subRegionTiles[0].y;
                            
                            int64_t sumX = 0, sumY = 0;
                            
                            for (const auto& t : subRegionTiles)
                            {
                                region.minX = std::min(region.minX, t.x);
                                region.maxX = std::max(region.maxX, t.x);
                                region.minY = std::min(region.minY, t.y);
                                region.maxY = std::max(region.maxY, t.y);
                                
                                sumX += t.x;
                                sumY += t.y;
                            }
                            
                            region.centerPoint = World::TilePos2(
                                static_cast<int16_t>(sumX / subRegionTiles.size()),
                                static_cast<int16_t>(sumY / subRegionTiles.size())
                            );
                            
                            // Assign region ID based on position in _regions vector
                            region.regionId = _regions.size();
                            
                            // Add to regions list
                            _regions.push_back(region);
                            
                            // Map all tiles to this region
                            for (const auto& t : subRegionTiles)
                            {
                                _tileToRegion[getTileKey(t)] = region.regionId;
                            }
                        }
                        // Note: Small regions (< kMinRegionTiles) are skipped here
                        // They will be detected as channels or handled via greedy navigation
                    }
                }
                else
                {
                    // Pure water cell - create a single region (only if large enough)
                    if (static_cast<int32_t>(tiles.size()) >= kMinRegionTiles)
                    {
                        WaterRegion region;
                        region.waterLevel = waterLevel;
                        region.tiles = tiles;
                        
                        region.minX = cellMinX;
                        region.maxX = cellMaxX;
                        region.minY = cellMinY;
                        region.maxY = cellMaxY;
                        
                        int64_t sumX = 0, sumY = 0;
                        
                        for (const auto& tile : tiles)
                        {
                            sumX += tile.x;
                            sumY += tile.y;
                        }
                        
                        region.centerPoint = World::TilePos2(
                            static_cast<int16_t>(sumX / tiles.size()),
                            static_cast<int16_t>(sumY / tiles.size())
                        );
                        
                        // Assign region ID based on position in _regions vector
                        region.regionId = _regions.size();
                        
                        // Add to regions list
                        _regions.push_back(region);
                        
                        // Map all tiles to this region
                        for (const auto& tile : tiles)
                        {
                            _tileToRegion[getTileKey(tile)] = region.regionId;
                        }
                    }
                    // Note: Small pure water cells (< kMinRegionTiles) are skipped
                    // They will be detected as channels or handled via greedy navigation
                }
            }
        }
        
        // Detect all water regions
        static void detectRegions()
        {
            Diagnostics::Logging::info("WaterWaypointNetwork: Detecting water regions...");
            
            _regions.clear();
            _tileToRegion.clear();
            
            std::unordered_set<uint32_t> visited;
            uint32_t operationCount = 0;
            uint32_t waterTilesFound = 0;
            uint32_t channelTilesSkipped = 0;
            uint32_t regionStartsFound = 0;
            
            // Sample every 4th tile to find region start points
            for (int32_t y = 0; y < World::kMapRows; y += 4)
            {
                for (int32_t x = 0; x < World::kMapColumns; x += 4)
                {
                    operationCount++;
                    World::TilePos2 pos(x, y);
                    
                    if (visited.count(getTileKey(pos)) > 0)
                        continue;
                    
                    auto tile = World::TileManager::get(pos);
                    auto* surface = tile.surface();
                    
                    if (surface == nullptr || surface->water() == 0)
                        continue;
                    
                    waterTilesFound++;
                    
                    // Skip narrow channels
                    if (isNarrowChannel(pos, surface->water()))
                    {
                        channelTilesSkipped++;
                        continue;
                    }
                    
                    // Found an open water tile - flood fill to get the whole region
                    // (flood fill may create multiple subdivided regions)
                    regionStartsFound++;
                    floodFillRegion(pos, surface->water(), visited);
                }
            }
            
            Diagnostics::Logging::info("WaterWaypointNetwork: Found {} water regions from {} region starts ({} water tiles sampled, {} channels skipped)", 
                _regions.size(), regionStartsFound, waterTilesFound, channelTilesSkipped);
            RoutingMetrics::recordWaterPathfindCall(operationCount);
        }
        
        // Trace a channel path from a starting point
        static std::vector<World::TilePos2> traceChannel(World::TilePos2 start, World::MicroZ waterLevel,
                                                          std::unordered_set<uint32_t>& visited)
        {
            std::vector<World::TilePos2> channelPath;
            channelPath.push_back(start);
            visited.insert(getTileKey(start));
            
            static const World::TilePos2 kDirections[] = {
                {0, -1}, {1, 0}, {0, 1}, {-1, 0}
            };
            
            World::TilePos2 current = start;
            
            // Follow the channel as long as it remains narrow
            while (true)
            {
                World::TilePos2 next = {-1, -1};
                int waterNeighbors = 0;
                
                for (const auto& dir : kDirections)
                {
                    World::TilePos2 neighbor = current + dir;
                    
                    if (!isWaterTile(neighbor, waterLevel))
                        continue;
                    
                    waterNeighbors++;
                    
                    // Skip already visited tiles
                    if (visited.count(getTileKey(neighbor)) > 0)
                        continue;
                    
                    // This could be the next tile in the channel
                    if (isNarrowChannel(neighbor, waterLevel))
                    {
                        next = neighbor;
                    }
                }
                
                // If no valid next tile, we've reached the end of the channel
                if (next.x == -1)
                    break;
                
                // Add to path and continue
                channelPath.push_back(next);
                visited.insert(getTileKey(next));
                current = next;
            }
            
            return channelPath;
        }
        
        // Detect channels connecting regions
        static void detectChannels()
        {
            Diagnostics::Logging::info("WaterWaypointNetwork: Detecting channels...");
            
            _channels.clear();
            std::unordered_set<uint32_t> visitedChannelTiles;
            uint32_t operationCount = 0;
            
            // Scan map looking for narrow channel tiles
            for (int32_t y = 0; y < World::kMapRows; y += 2)
            {
                for (int32_t x = 0; x < World::kMapColumns; x += 2)
                {
                    operationCount++;
                    World::TilePos2 pos(x, y);
                    
                    if (visitedChannelTiles.count(getTileKey(pos)) > 0)
                        continue;
                    
                    auto tile = World::TileManager::get(pos);
                    auto* surface = tile.surface();
                    
                    if (surface == nullptr || surface->water() == 0)
                        continue;
                    
                    // Check if this is a narrow channel
                    if (!isNarrowChannel(pos, surface->water()))
                        continue;
                    
                    // Trace the channel path
                    auto channelPath = traceChannel(pos, surface->water(), visitedChannelTiles);
                    
                    // Only keep channels that are at least 3 tiles long
                    if (channelPath.size() < 3)
                        continue;
                    
                    // Determine which regions this channel connects
                    // Check regions at both ends of the channel
                    static const World::TilePos2 kDirections[] = {
                        {0, -1}, {1, 0}, {0, 1}, {-1, 0}
                    };
                    
                    uint16_t regionA = 0xFFFF;
                    uint16_t regionB = 0xFFFF;
                    
                    // Check start of channel
                    for (const auto& dir : kDirections)
                    {
                        World::TilePos2 neighbor = channelPath.front() + dir;
                        auto it = _tileToRegion.find(getTileKey(neighbor));
                        if (it != _tileToRegion.end())
                        {
                            regionA = it->second;
                            break;
                        }
                    }
                    
                    // Check end of channel
                    for (const auto& dir : kDirections)
                    {
                        World::TilePos2 neighbor = channelPath.back() + dir;
                        auto it = _tileToRegion.find(getTileKey(neighbor));
                        if (it != _tileToRegion.end())
                        {
                            regionB = it->second;
                            break;
                        }
                    }
                    
                    // Create channel if it connects regions
                    if (regionA != 0xFFFF && regionB != 0xFFFF && regionA != regionB)
                    {
                        WaterChannel channel;
                        channel.path = channelPath;
                        channel.waterLevel = surface->water();
                        channel.regionA = regionA;
                        channel.regionB = regionB;
                        
                        uint16_t channelId = static_cast<uint16_t>(_channels.size());
                        _channels.push_back(channel);
                        
                        // Update regions with channel connections
                        _regions[regionA].connectedChannels.push_back(channelId);
                        _regions[regionA].connectedRegions.push_back(regionB);
                        _regions[regionB].connectedChannels.push_back(channelId);
                        _regions[regionB].connectedRegions.push_back(regionA);
                        
                        Diagnostics::Logging::verbose("WaterWaypointNetwork: Found channel {} connecting regions {} and {} ({} tiles)", 
                            channelId, regionA, regionB, channelPath.size());
                    }
                }
            }
            
            Diagnostics::Logging::info("WaterWaypointNetwork: Found {} channels", _channels.size());
            RoutingMetrics::recordWaterPathfindCall(operationCount);
        }

        // Detect adjacent regions and connect them
        static void detectAdjacentRegions()
        {
            Diagnostics::Logging::info("WaterWaypointNetwork: Detecting adjacent regions...");
            
            uint32_t adjacencyCount = 0;
            
            // For each region, check if any of its tiles are adjacent to tiles from other regions
            for (auto& region : _regions)
            {
                // Check tiles on the boundary of this region
                for (const auto& tile : region.tiles)
                {
                    // Check all 4 directions
                    static const World::TilePos2 kDirections[] = {
                        {0, -1}, {1, 0}, {0, 1}, {-1, 0}
                    };
                    
                    for (const auto& dir : kDirections)
                    {
                        World::TilePos2 neighbor = tile + dir;
                        auto it = _tileToRegion.find(getTileKey(neighbor));
                        
                        if (it != _tileToRegion.end() && it->second != region.regionId)
                        {
                            uint16_t neighborRegionId = it->second;
                            
                            // Check if this connection already exists
                            bool alreadyConnected = false;
                            for (uint16_t connectedId : region.connectedRegions)
                            {
                                if (connectedId == neighborRegionId)
                                {
                                    alreadyConnected = true;
                                    break;
                                }
                            }
                            
                            if (!alreadyConnected)
                            {
                                // Add bidirectional connection
                                region.connectedRegions.push_back(neighborRegionId);
                                _regions[neighborRegionId].connectedRegions.push_back(region.regionId);
                                adjacencyCount++;
                            }
                        }
                    }
                }
            }
            
            Diagnostics::Logging::info("WaterWaypointNetwork: Found {} adjacent region connections", adjacencyCount);
        }

        void initialize()
        {
            Diagnostics::Logging::info("WaterWaypointNetwork: Initializing region/channel network...");
            
            // Step 1: Detect open water regions (creates small 16x16 subdivisions for convexity)
            detectRegions();
            
            // Step 2: Detect channels between regions
            detectChannels();
            
            // Step 3: Detect adjacent regions (connects neighboring regions)
            detectAdjacentRegions();
            
            Diagnostics::Logging::info("WaterWaypointNetwork: Initialization complete - {} regions, {} channels", 
                _regions.size(), _channels.size());

            _initialized = true;
            _dirty = false;
        }

        void ensureInitialized()
        {
            if (!_initialized || _dirty)
            {
                Diagnostics::Logging::info("WaterWaypointNetwork: ensureInitialized called (_initialized={}, _dirty={})", _initialized, _dirty);
                initialize();
            }
        }

        void markDirty()
        {
            _dirty = true;
        }
        
        void reset()
        {
            Diagnostics::Logging::info("WaterWaypointNetwork: Resetting network data");
            _regions.clear();
            _channels.clear();
            _tileToRegion.clear();
            _initialized = false;
            _dirty = true;
        }

        uint16_t findRegionAt(World::TilePos2 pos, World::MicroZ waterLevel)
        {
            ensureInitialized();
            
            auto it = _tileToRegion.find(getTileKey(pos));
            if (it != _tileToRegion.end())
            {
                // Verify water level matches
                if (_regions[it->second].waterLevel == waterLevel)
                    return it->second;
            }
            
            // If exact position not in a region, search nearby tiles (for docks/waypoints)
            // This allows pathfinding to connect to the nearest water region
            static const World::TilePos2 kSearchPattern[] = {
                {0, -1}, {1, 0}, {0, 1}, {-1, 0},  // Adjacent tiles
                {-1, -1}, {1, -1}, {1, 1}, {-1, 1}, // Diagonals
                {0, -2}, {2, 0}, {0, 2}, {-2, 0},  // 2 tiles away
            };
            
            for (const auto& offset : kSearchPattern)
            {
                auto nearbyPos = pos + offset;
                auto nearbyIt = _tileToRegion.find(getTileKey(nearbyPos));
                if (nearbyIt != _tileToRegion.end())
                {
                    if (_regions[nearbyIt->second].waterLevel == waterLevel)
                    {
                        // Cache this result for future lookups
                        _tileToRegion[getTileKey(pos)] = nearbyIt->second;
                        return nearbyIt->second;
                    }
                }
            }
            
            return 0xFFFF;
        }

        const WaterRegion* getRegion(uint16_t regionId)
        {
            ensureInitialized();

            if (regionId >= _regions.size())
                return nullptr;

            return &_regions[regionId];
        }

        const std::vector<WaterRegion>& getAllRegions()
        {
            ensureInitialized();
            return _regions;
        }

        const WaterChannel* getChannel(uint16_t channelId)
        {
            ensureInitialized();

            if (channelId >= _channels.size())
                return nullptr;

            return &_channels[channelId];
        }

        const std::vector<WaterChannel>& getAllChannels()
        {
            ensureInitialized();
            return _channels;
        }
    }
}
