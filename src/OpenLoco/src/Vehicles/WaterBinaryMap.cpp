#include "WaterBinaryMap.h"
#include "Map/TileManager.h"
#include "Map/SurfaceElement.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
#include <unordered_set>

using namespace OpenLoco::World;

namespace OpenLoco::Vehicles::WaterBinaryMap
{
    namespace
    {
        // Map dimensions
        static uint32_t _paddedSize = 0;
        static uint32_t _numLevels = 0;

        // Bitmap: _paddedSize rows, each row has _paddedSize bits packed into uint64_t
        // For a 512x512 map: 512 rows * 8 uint64_t = 32KB
        static std::vector<std::vector<uint64_t>> _waterBits;

        // Quadrant cache: level -> 2D array of water counts
        // Level 0 = 1 quadrant (whole map)
        // Level numLevels-1 = finest level (2x2 tiles per quadrant)
        static std::vector<std::vector<std::vector<uint32_t>>> _quadrantCache;

        static bool _initialized = false;
        static bool _dirty = true;
        
        // Incremental update state - continuous spiral pattern
        static int32_t _updateSpiralX = 0;
        static int32_t _updateSpiralY = 0;
        static int32_t _updateSpiralRadius = 0;
        static int32_t _updateSpiralSide = 0; // 0=right, 1=down, 2=left, 3=up
        static int32_t _updateSpiralSteps = 0;
        
        // Priority spirals (e.g., when ship gets stuck)
        struct PrioritySpiral
        {
            int32_t centerX, centerY;
            int32_t currentX, currentY;
            int32_t radius;
            int32_t side;
            int32_t steps;
            int32_t maxRadius;
            uint32_t tickCreated; // To prevent re-triggering too soon
        };
        static std::vector<PrioritySpiral> _prioritySpirals;
        static uint32_t _currentTick = 0;
        
        // Debug: track recently updated tiles for visualization
        struct RecentUpdate
        {
            int16_t x, y;
            uint32_t tick;
        };
        static std::vector<RecentUpdate> _recentUpdates;
        constexpr size_t kMaxRecentUpdates = 64; // Track last 64 tiles updated

        // Compute next power of 2 >= v
        static uint32_t nextPowerOf2(uint32_t v)
        {
            if (v == 0)
                return 1;
            v--;
            v |= v >> 1;
            v |= v >> 2;
            v |= v >> 4;
            v |= v >> 8;
            v |= v >> 16;
            return v + 1;
        }



        // Build the bitmap from tile data
        static void buildBitmap()
        {
            // Determine padded size based on actual map dimensions
            uint32_t maxDim = std::max(static_cast<uint32_t>(kMapRows), static_cast<uint32_t>(kMapColumns));
            _paddedSize = nextPowerOf2(maxDim);
            _numLevels = static_cast<uint32_t>(std::log2(_paddedSize)) + 1;

            Diagnostics::Logging::info("WaterBinaryMap: Map {}x{}, padded to {}, {} levels",
                                       kMapColumns, kMapRows, _paddedSize, _numLevels);

            // Allocate bitmap
            uint32_t wordsPerRow = _paddedSize / 64;
            if (wordsPerRow == 0)
                wordsPerRow = 1; // Handle small maps

            _waterBits.assign(_paddedSize, std::vector<uint64_t>(wordsPerRow, 0));

            // Scan all tiles and set bits for water
            uint32_t waterTileCount = 0;
            for (int16_t y = 0; y < kMapRows; ++y)
            {
                for (int16_t x = 0; x < kMapColumns; ++x)
                {
                    auto tile = TileManager::get(TilePos2(x, y));
                    auto* surface = tile.surface();
                    if (surface != nullptr && surface->water() > 0)
                    {
                        _waterBits[y][x >> 6] |= (1ULL << (x & 63));
                        waterTileCount++;
                    }
                }
            }

            Diagnostics::Logging::info("WaterBinaryMap: Found {} water tiles", waterTileCount);
        }

        // Build quadrant cache bottom-up from bitmap
        // Level 0 = finest (2x2 tiles per quadrant, most quadrants)
        // Level numLevels-1 = coarsest (1 quadrant = whole map)
        static void buildQuadrantCache()
        {
            // Allocate cache for all levels
            _quadrantCache.resize(_numLevels);

            // Level 0: finest level with 2x2 tile quadrants
            // For 512x512 map: 256x256 quadrants at level 0
            uint32_t quadrantsPerSide = _paddedSize / 2;

            _quadrantCache[0].assign(quadrantsPerSide, std::vector<uint32_t>(quadrantsPerSide, 0));

            for (uint32_t qy = 0; qy < quadrantsPerSide; ++qy)
            {
                for (uint32_t qx = 0; qx < quadrantsPerSide; ++qx)
                {
                    // Count water tiles in this 2x2 block
                    uint32_t count = 0;
                    uint32_t baseX = qx * 2;
                    uint32_t baseY = qy * 2;

                    for (uint32_t dy = 0; dy < 2; ++dy)
                    {
                        for (uint32_t dx = 0; dx < 2; ++dx)
                        {
                            uint32_t tx = baseX + dx;
                            uint32_t ty = baseY + dy;
                            if (ty < _paddedSize && tx < _paddedSize)
                            {
                                uint32_t wordIdx = tx >> 6;
                                uint32_t bitIdx = tx & 63;
                                if (wordIdx < _waterBits[ty].size())
                                {
                                    if ((_waterBits[ty][wordIdx] >> bitIdx) & 1)
                                    {
                                        count++;
                                    }
                                }
                            }
                        }
                    }
                    _quadrantCache[0][qy][qx] = count;
                }
            }

            // Build coarser levels bottom-up
            // Each coarser level has half the quadrants per side
            for (uint32_t level = 1; level < _numLevels; ++level)
            {
                uint32_t prevQuadrants = quadrantsPerSide >> (level - 1);
                uint32_t numQuadrants = quadrantsPerSide >> level;
                
                if (numQuadrants == 0)
                    numQuadrants = 1;

                _quadrantCache[level].assign(numQuadrants, std::vector<uint32_t>(numQuadrants, 0));

                for (uint32_t qy = 0; qy < numQuadrants; ++qy)
                {
                    for (uint32_t qx = 0; qx < numQuadrants; ++qx)
                    {
                        // Sum 4 children from level-1
                        uint32_t childX = qx * 2;
                        uint32_t childY = qy * 2;
                        uint32_t childLevel = level - 1;

                        uint32_t sum = 0;
                        if (childY < prevQuadrants && childX < prevQuadrants)
                            sum += _quadrantCache[childLevel][childY][childX];
                        if (childY < prevQuadrants && childX + 1 < prevQuadrants)
                            sum += _quadrantCache[childLevel][childY][childX + 1];
                        if (childY + 1 < prevQuadrants && childX < prevQuadrants)
                            sum += _quadrantCache[childLevel][childY + 1][childX];
                        if (childY + 1 < prevQuadrants && childX + 1 < prevQuadrants)
                            sum += _quadrantCache[childLevel][childY + 1][childX + 1];

                        _quadrantCache[level][qy][qx] = sum;
                    }
                }
            }

            Diagnostics::Logging::info("WaterBinaryMap: Quadrant cache built, {} levels", _numLevels);
        }

        // Get tiles per quadrant side at given level
        static uint32_t getTilesPerQuadrant(uint32_t level)
        {
            // Level 0 = finest = 2 tiles per quadrant
            // Level numLevels-1 = coarsest = whole map
            // tilesPerQuad = 2^(level+1)
            return 2U << level;
        }
        
        // Update a single tile in the bitmap and affected quadrants
        static void updateTile(int16_t x, int16_t y)
        {
            if (x < 0 || y < 0 || x >= kMapColumns || y >= kMapRows)
                return;
                
            if (!_initialized)
                return;
            
            // Track this update for debug visualization
            if (_recentUpdates.size() >= kMaxRecentUpdates)
            {
                _recentUpdates.erase(_recentUpdates.begin());
            }
            _recentUpdates.push_back({x, y, _currentTick});
            
            // Check current water state
            auto tile = TileManager::get(TilePos2(x, y));
            auto* surface = tile.surface();
            bool hasWater = (surface != nullptr && surface->water() > 0);
            
            // Get current bit state
            uint32_t ux = static_cast<uint32_t>(x);
            uint32_t uy = static_cast<uint32_t>(y);
            uint32_t wordIdx = ux >> 6;
            uint32_t bitIdx = ux & 63;
            
            if (uy >= _waterBits.size() || wordIdx >= _waterBits[uy].size())
                return;
                
            bool currentBit = (_waterBits[uy][wordIdx] >> bitIdx) & 1;
            
            // If unchanged, skip
            if (currentBit == hasWater)
                return;
            
            // Update bitmap
            if (hasWater)
            {
                _waterBits[uy][wordIdx] |= (1ULL << bitIdx);
            }
            else
            {
                _waterBits[uy][wordIdx] &= ~(1ULL << bitIdx);
            }
            
            // Update quadrant cache
            // Start from finest level (level 0) and propagate up
            int32_t delta = hasWater ? 1 : -1;
            
            for (uint32_t level = 0; level < _numLevels; ++level)
            {
                uint32_t tilesPerQuad = getTilesPerQuadrant(level);
                uint32_t qx = ux / tilesPerQuad;
                uint32_t qy = uy / tilesPerQuad;
                
                if (qy >= _quadrantCache[level].size() || qx >= _quadrantCache[level][qy].size())
                    break;
                    
                // Apply delta
                if (delta > 0)
                {
                    _quadrantCache[level][qy][qx]++;
                }
                else if (_quadrantCache[level][qy][qx] > 0)
                {
                    _quadrantCache[level][qy][qx]--;
                }
            }
        }
        
        // Advance spiral by one step, returns true if still valid
        static bool advanceSpiral(int32_t& offsetX, int32_t& offsetY, 
                                 int32_t& radius, int32_t& side, int32_t& steps, int32_t maxRadius)
        {
            if (radius > maxRadius)
                return false;
                
            // For radius 0, just process center
            if (radius == 0)
            {
                offsetX = 0;
                offsetY = 0;
                radius = 1;
                offsetX = -1;
                offsetY = -1;
                side = 0;
                steps = 0;
                return true;
            }
            
            // Move one step in current direction
            switch (side)
            {
                case 0: offsetX++; break; // right
                case 1: offsetY++; break; // down
                case 2: offsetX--; break; // left
                case 3: offsetY--; break; // up
            }
            
            steps++;
            
            // Check if we need to turn (spiral pattern logic)
            int32_t stepsInSide = (side == 0 || side == 2) ? (radius * 2) : (radius * 2);
            
            if (steps >= stepsInSide)
            {
                steps = 0;
                side++;
                
                if (side >= 4)
                {
                    side = 0;
                    radius++;
                    offsetX = -radius;
                    offsetY = -radius;
                    
                    if (radius > maxRadius)
                        return false;
                }
            }
            
            return true;
        }
    }

    void initialize()
    {
        Diagnostics::Logging::info("WaterBinaryMap: Initializing...");
        buildBitmap();
        buildQuadrantCache();
        _initialized = true;
        _dirty = false;
        
        // Initialize background spiral from center of map
        _updateSpiralX = kMapColumns / 2;
        _updateSpiralY = kMapRows / 2;
        _updateSpiralRadius = 0;
        _updateSpiralSide = 0;
        _updateSpiralSteps = 0;
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

    void reset()
    {
        Diagnostics::Logging::info("WaterBinaryMap: Resetting");
        _waterBits.clear();
        _quadrantCache.clear();
        _paddedSize = 0;
        _numLevels = 0;
        _initialized = false;
        _dirty = true;
        
        // Reset spiral state (will be properly initialized in initialize())
        _updateSpiralX = 0;
        _updateSpiralY = 0;
        _updateSpiralRadius = 0;
        _updateSpiralSide = 0;
        _updateSpiralSteps = 0;
        _prioritySpirals.clear();
        _currentTick = 0;
    }
    
    void update()
    {
        if (!_initialized)
            return;
            
        _currentTick++;
        constexpr int32_t kTilesPerFrame = 16;
        int32_t tilesUpdated = 0;
        
        // Process priority spirals first (when ships get stuck)
        while (!_prioritySpirals.empty() && tilesUpdated < kTilesPerFrame)
        {
            auto& spiral = _prioritySpirals.front();
            
            // Update current tile
            int16_t worldX = static_cast<int16_t>(spiral.centerX + spiral.currentX);
            int16_t worldY = static_cast<int16_t>(spiral.centerY + spiral.currentY);
            updateTile(worldX, worldY);
            tilesUpdated++;
            
            // Advance spiral
            if (!advanceSpiral(spiral.currentX, spiral.currentY, 
                              spiral.radius, spiral.side, spiral.steps, spiral.maxRadius))
            {
                // This spiral is done, remove it
                _prioritySpirals.erase(_prioritySpirals.begin());
            }
        }
        
        // Fill remaining budget with continuous background spiral
        int32_t centerX = kMapColumns / 2;
        int32_t centerY = kMapRows / 2;
        int32_t maxDim = std::max(static_cast<int32_t>(kMapRows), static_cast<int32_t>(kMapColumns));
        
        while (tilesUpdated < kTilesPerFrame)
        {
            // Calculate world position from center + offset
            int16_t worldX = static_cast<int16_t>(centerX + _updateSpiralX);
            int16_t worldY = static_cast<int16_t>(centerY + _updateSpiralY);
            updateTile(worldX, worldY);
            tilesUpdated++;
            
            // Advance spiral
            if (!advanceSpiral(_updateSpiralX, _updateSpiralY, 
                              _updateSpiralRadius, _updateSpiralSide, _updateSpiralSteps, maxDim / 2))
            {
                // Restart from center when we've covered the whole map
                _updateSpiralX = 0;
                _updateSpiralY = 0;
                _updateSpiralRadius = 0;
                _updateSpiralSide = 0;
                _updateSpiralSteps = 0;
            }
        }
    }
    
    void refreshAroundPosition(World::TilePos2 pos)
    {
        if (!_initialized)
            return;
            
        constexpr uint32_t kMinTicksBetweenRefresh = 60; // ~1 second at 60 ticks/sec
        
        // Add a priority spiral centered at this position
        // Check if we already have a recent spiral near this location to avoid duplicates
        for (const auto& spiral : _prioritySpirals)
        {
            int32_t dx = spiral.centerX - pos.x;
            int32_t dy = spiral.centerY - pos.y;
            if (dx * dx + dy * dy < 100) // Within ~10 tiles
            {
                // Check if it was created recently
                if (_currentTick - spiral.tickCreated < kMinTicksBetweenRefresh)
                {
                    return; // Already have a recent spiral nearby
                }
            }
        }
        
        PrioritySpiral newSpiral;
        newSpiral.centerX = pos.x;
        newSpiral.centerY = pos.y;
        newSpiral.currentX = 0;
        newSpiral.currentY = 0;
        newSpiral.radius = 0;
        newSpiral.side = 0;
        newSpiral.steps = 0;
        newSpiral.maxRadius = 32; // Refresh 32 tiles around the stuck position
        newSpiral.tickCreated = _currentTick;
        
        _prioritySpirals.push_back(newSpiral);
        
        Diagnostics::Logging::info("WaterBinaryMap: Added priority refresh spiral at ({}, {})", pos.x, pos.y);
    }

    bool isWater(TilePos2 pos, [[maybe_unused]] MicroZ waterLevel)
    {
        ensureInitialized();

        if (pos.x < 0 || pos.y < 0 ||
            static_cast<uint32_t>(pos.x) >= _paddedSize ||
            static_cast<uint32_t>(pos.y) >= _paddedSize)
        {
            return false;
        }

        uint32_t wordIdx = static_cast<uint32_t>(pos.x) >> 6;
        uint32_t bitIdx = static_cast<uint32_t>(pos.x) & 63;

        if (wordIdx >= _waterBits[pos.y].size())
        {
            return false;
        }

        return (_waterBits[pos.y][wordIdx] >> bitIdx) & 1;
    }

    uint32_t getQuadrantWaterCount(uint32_t level, uint32_t qx, uint32_t qy)
    {
        ensureInitialized();

        if (level >= _numLevels)
            return 0;

        // Level 0 has _paddedSize/2 quadrants per side
        // Each higher level has half as many
        uint32_t numQuadrants = (_paddedSize / 2) >> level;
        if (numQuadrants == 0)
            numQuadrants = 1;
            
        if (qx >= numQuadrants || qy >= numQuadrants)
            return 0;

        return _quadrantCache[level][qy][qx];
    }

    bool isPureWater(uint32_t level, uint32_t qx, uint32_t qy)
    {
        uint32_t tilesPerQuad = getTilesPerQuadrant(level);
        uint32_t maxTiles = tilesPerQuad * tilesPerQuad;
        return getQuadrantWaterCount(level, qx, qy) == maxTiles;
    }

    bool isAllLand(uint32_t level, uint32_t qx, uint32_t qy)
    {
        return getQuadrantWaterCount(level, qx, qy) == 0;
    }

    // Check if a tile is navigable water at 1x1 level
    static bool isTileWater(int32_t tx, int32_t ty)
    {
        if (tx < 0 || ty < 0 || tx >= static_cast<int32_t>(kMapColumns) || ty >= static_cast<int32_t>(kMapRows))
            return false;
        
        uint32_t ux = static_cast<uint32_t>(tx);
        uint32_t uy = static_cast<uint32_t>(ty);
        
        if (uy >= _waterBits.size() || (ux >> 6) >= _waterBits[uy].size())
            return false;
            
        return (_waterBits[uy][ux >> 6] >> (ux & 63)) & 1;
    }

    // Hierarchical pathfinding - lazy subdivision from coarse to fine
    // Key insight: only subdivide mixed quadrants, pure water/land quadrants don't need subdivision
    PathResult findPath(TilePos2 from, TilePos2 to, MicroZ waterLevel)
    {
        ensureInitialized();

        PathResult result;
        result.hasPath = false;

        // Quick validation at tile level
        if (!isWater(from, waterLevel) || !isWater(to, waterLevel))
        {
            Diagnostics::Logging::warn("BSP: Start or end tile is not water!");
            return result;
        }

        int32_t dx = std::abs(from.x - to.x);
        int32_t dy = std::abs(from.y - to.y);
        int32_t distance = dx + dy;

        // For very short distances, just return target directly
        if (distance <= 4)
        {
            result.hasPath = true;
            result.waypoints.push_back(to);
            return result;
        }

        Diagnostics::Logging::info("BSP pathfind: from ({},{}) to ({},{}), distance={}",
            from.x, from.y, to.x, to.y, distance);

        // BFS at tile level (1x1) for guaranteed accuracy
        // This is the "lazy" approach - we work at tile level directly
        // The bitmap makes tile queries O(1), so this is fast
        
        struct TileNode
        {
            int16_t x, y;
            int16_t parentX, parentY;
            bool hasParent;
        };

        auto tileKey = [](int16_t x, int16_t y) -> uint32_t
        {
            return (static_cast<uint32_t>(static_cast<uint16_t>(y)) << 16) | static_cast<uint16_t>(x);
        };

        std::queue<TileNode> openSet;
        std::unordered_set<uint32_t> visited;
        std::unordered_map<uint32_t, TileNode> allNodes;

        TileNode startNode = {from.x, from.y, 0, 0, false};
        openSet.push(startNode);
        visited.insert(tileKey(from.x, from.y));
        allNodes[tileKey(from.x, from.y)] = startNode;

        bool found = false;
        TileNode endNode;

        // Direction offsets for 4-connectivity
        static const int16_t dtx[] = {0, 1, 0, -1};
        static const int16_t dty[] = {-1, 0, 1, 0};

        // Limit search to prevent infinite loops on very large maps
        constexpr size_t kMaxSearchNodes = 100000;
        size_t nodesExplored = 0;

        while (!openSet.empty() && !found && nodesExplored < kMaxSearchNodes)
        {
            TileNode current = openSet.front();
            openSet.pop();
            nodesExplored++;

            if (current.x == to.x && current.y == to.y)
            {
                found = true;
                endNode = current;
                break;
            }

            // Explore neighbors
            for (int dir = 0; dir < 4; ++dir)
            {
                int16_t nx = current.x + dtx[dir];
                int16_t ny = current.y + dty[dir];

                uint32_t key = tileKey(nx, ny);
                if (visited.count(key) > 0)
                {
                    continue;
                }

                // Check if tile is water using our bitmap (O(1))
                if (!isTileWater(nx, ny))
                {
                    continue;
                }

                visited.insert(key);
                TileNode neighbor = {nx, ny, current.x, current.y, true};
                openSet.push(neighbor);
                allNodes[key] = neighbor;
            }
        }

        if (!found)
        {
            Diagnostics::Logging::warn("BSP: NO PATH at tile level! Explored {} tiles", nodesExplored);
            return result;
        }
        
        Diagnostics::Logging::info("BSP: Path found! Explored {} tiles", nodesExplored);

        // Reconstruct path
        std::vector<TilePos2> path;
        TileNode node = endNode;

        while (true)
        {
            path.push_back(TilePos2{node.x, node.y});

            if (!node.hasParent)
            {
                break;
            }

            uint32_t parentKey = tileKey(node.parentX, node.parentY);
            auto it = allNodes.find(parentKey);
            if (it == allNodes.end())
                break;
            node = it->second;
        }

        // Reverse to get path from start to end
        std::reverse(path.begin(), path.end());

        // Store full path for debug visualization
        result.fullPath = path;

        // Simplify path: only keep waypoints where direction changes significantly
        // or every N tiles to keep the ship on track
        constexpr size_t kWaypointInterval = 8;
        for (size_t i = 1; i < path.size(); ++i)
        {
            // Always add final destination
            if (i == path.size() - 1)
            {
                result.waypoints.push_back(path[i]);
            }
            // Add waypoints at intervals
            else if (i % kWaypointInterval == 0)
            {
                result.waypoints.push_back(path[i]);
            }
        }

        // Ensure we have at least the destination
        if (result.waypoints.empty())
        {
            result.waypoints.push_back(to);
        }

        result.hasPath = true;

        Diagnostics::Logging::info("BSP: Simplified to {} waypoints", result.waypoints.size());

        return result;
    }

    uint32_t getPaddedSize()
    {
        ensureInitialized();
        return _paddedSize;
    }

    uint32_t getNumLevels()
    {
        ensureInitialized();
        return _numLevels;
    }
    
    std::vector<World::TilePos2> getRecentlyUpdatedTiles(uint32_t maxAgeTicks)
    {
        std::vector<World::TilePos2> result;
        
        for (const auto& update : _recentUpdates)
        {
            if (_currentTick - update.tick <= maxAgeTicks)
            {
                result.push_back(TilePos2{update.x, update.y});
            }
        }
        
        return result;
    }
}
