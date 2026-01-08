#include "WaterNetwork.h"
#include "Map/TileManager.h"
#include "Map/SurfaceElement.h"
#include <OpenLoco/Diagnostics/Logging.h>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <unordered_map>

using namespace OpenLoco::World;

namespace OpenLoco::Pathfinding
{
    namespace
    {
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

        // Get tiles per quadrant side at given level
        static uint32_t getTilesPerQuadrant(uint32_t level)
        {
            return 2U << level;
        }
    }

    // WaterBitmap implementation
    void WaterBitmap::build(MicroZ waterLevel)
    {
        uint32_t maxDim = std::max(static_cast<uint32_t>(kMapRows), static_cast<uint32_t>(kMapColumns));
        paddedSize = nextPowerOf2(maxDim);
        numLevels = static_cast<uint32_t>(std::log2(paddedSize)) + 1;

        uint32_t wordsPerRow = paddedSize / 64;
        if (wordsPerRow == 0)
            wordsPerRow = 1;

        bits.assign(paddedSize, std::vector<uint64_t>(wordsPerRow, 0));

        uint32_t waterTileCount = 0;
        for (int16_t y = 0; y < kMapRows; ++y)
        {
            for (int16_t x = 0; x < kMapColumns; ++x)
            {
                auto tile = TileManager::get(TilePos2(x, y));
                auto* surface = tile.surface();
                if (surface != nullptr && surface->water() > 0)
                {
                    // Check if this tile's water level matches
                    auto tileWaterLevel = surface->water();
                    if (tileWaterLevel == waterLevel)
                    {
                        bits[y][x >> 6] |= (1ULL << (x & 63));
                        waterTileCount++;
                    }
                }
            }
        }

        Diagnostics::Logging::info("WaterBitmap: Built for level {}, {} water tiles", waterLevel, waterTileCount);
        dirty = false;
    }

    void WaterBitmap::buildQuadrantCache()
    {
        quadrantCache.resize(numLevels);

        uint32_t quadrantsPerSide = paddedSize / 2;
        quadrantCache[0].assign(quadrantsPerSide, std::vector<uint32_t>(quadrantsPerSide, 0));

        for (uint32_t qy = 0; qy < quadrantsPerSide; ++qy)
        {
            for (uint32_t qx = 0; qx < quadrantsPerSide; ++qx)
            {
                uint32_t count = 0;
                uint32_t baseX = qx * 2;
                uint32_t baseY = qy * 2;

                for (uint32_t dy = 0; dy < 2; ++dy)
                {
                    for (uint32_t dx = 0; dx < 2; ++dx)
                    {
                        uint32_t tx = baseX + dx;
                        uint32_t ty = baseY + dy;
                        if (ty < paddedSize && tx < paddedSize)
                        {
                            uint32_t wordIdx = tx >> 6;
                            uint32_t bitIdx = tx & 63;
                            if (wordIdx < bits[ty].size())
                            {
                                if ((bits[ty][wordIdx] >> bitIdx) & 1)
                                {
                                    count++;
                                }
                            }
                        }
                    }
                }
                quadrantCache[0][qy][qx] = count;
            }
        }

        for (uint32_t level = 1; level < numLevels; ++level)
        {
            uint32_t prevQuadrants = quadrantsPerSide >> (level - 1);
            uint32_t numQuadrants = quadrantsPerSide >> level;

            if (numQuadrants == 0)
                numQuadrants = 1;

            quadrantCache[level].assign(numQuadrants, std::vector<uint32_t>(numQuadrants, 0));

            for (uint32_t qy = 0; qy < numQuadrants; ++qy)
            {
                for (uint32_t qx = 0; qx < numQuadrants; ++qx)
                {
                    uint32_t childX = qx * 2;
                    uint32_t childY = qy * 2;
                    uint32_t childLevel = level - 1;

                    uint32_t sum = 0;
                    if (childY < prevQuadrants && childX < prevQuadrants)
                        sum += quadrantCache[childLevel][childY][childX];
                    if (childY < prevQuadrants && childX + 1 < prevQuadrants)
                        sum += quadrantCache[childLevel][childY][childX + 1];
                    if (childY + 1 < prevQuadrants && childX < prevQuadrants)
                        sum += quadrantCache[childLevel][childY + 1][childX];
                    if (childY + 1 < prevQuadrants && childX + 1 < prevQuadrants)
                        sum += quadrantCache[childLevel][childY + 1][childX + 1];

                    quadrantCache[level][qy][qx] = sum;
                }
            }
        }
    }

    bool WaterBitmap::isWater(TilePos2 pos) const
    {
        if (pos.x < 0 || pos.y < 0 ||
            static_cast<uint32_t>(pos.x) >= paddedSize ||
            static_cast<uint32_t>(pos.y) >= paddedSize)
        {
            return false;
        }

        uint32_t wordIdx = static_cast<uint32_t>(pos.x) >> 6;
        uint32_t bitIdx = static_cast<uint32_t>(pos.x) & 63;

        if (wordIdx >= bits[pos.y].size())
        {
            return false;
        }

        return (bits[pos.y][wordIdx] >> bitIdx) & 1;
    }

    uint32_t WaterBitmap::getQuadrantWaterCount(uint32_t level, uint32_t qx, uint32_t qy) const
    {
        if (level >= numLevels)
            return 0;

        uint32_t numQuadrants = (paddedSize / 2) >> level;
        if (numQuadrants == 0)
            numQuadrants = 1;

        if (qx >= numQuadrants || qy >= numQuadrants)
            return 0;

        return quadrantCache[level][qy][qx];
    }

    // WaterNetwork implementation
    void WaterNetwork::initialize()
    {
        Diagnostics::Logging::info("WaterNetwork: Initializing...");
        scanWaterLevels();
        _initialized = true;
        _dirty = false;
        Diagnostics::Logging::info("WaterNetwork: Found {} water levels", _waterLevels.size());
    }

    void WaterNetwork::reset()
    {
        Diagnostics::Logging::info("WaterNetwork: Resetting");
        _waterLevels.clear();
        _initialized = false;
        _dirty = true;
        _ticksSinceRefresh = 0;
        _ticksAtGoodFps = 0;
        _rebuildRowIndex = 0;
        _rebuildInProgress = false;
    }

    void WaterNetwork::markDirty()
    {
        _dirty = true;
        for (auto& [level, bitmap] : _waterLevels)
        {
            bitmap.dirty = true;
        }
    }

    void WaterNetwork::ensureInitialized()
    {
        if (!_initialized || _dirty)
        {
            initialize();
        }
    }

    void WaterNetwork::updatePeriodicRefresh()
    {
        if (!_initialized)
            return;

        _ticksSinceRefresh++;
        _ticksAtGoodFps++;

        // Lazy incremental rebuild if dirty
        if (_dirty || _rebuildInProgress)
        {
            rebuildIncremental();
            return;
        }

        bool shouldRefresh = false;

        if (_ticksAtGoodFps >= kFastRefreshInterval && _ticksSinceRefresh >= kFastRefreshInterval)
        {
            Diagnostics::Logging::info("WaterNetwork: Periodic refresh (1 min at good FPS)");
            shouldRefresh = true;
        }
        else if (_ticksSinceRefresh >= kSlowRefreshInterval)
        {
            Diagnostics::Logging::info("WaterNetwork: Periodic refresh (10 min)");
            shouldRefresh = true;
        }

        if (shouldRefresh)
        {
            markDirty();
            _ticksSinceRefresh = 0;
            _ticksAtGoodFps = 0;
        }
    }

    void WaterNetwork::scanWaterLevels()
    {
        _waterLevels.clear();

        // Scan map for all water levels
        std::unordered_set<MicroZ> foundLevels;
        for (int16_t y = 0; y < kMapRows; ++y)
        {
            for (int16_t x = 0; x < kMapColumns; ++x)
            {
                auto tile = TileManager::get(TilePos2(x, y));
                auto* surface = tile.surface();
                if (surface != nullptr && surface->water() > 0)
                {
                    foundLevels.insert(surface->water());
                }
            }
        }

        // Build bitmap for each level
        for (auto level : foundLevels)
        {
            _waterLevels[level] = WaterBitmap{};
            _waterLevels[level].build(level);
            _waterLevels[level].buildQuadrantCache();
        }
    }

    void WaterNetwork::rebuildIncremental()
    {
        if (!_rebuildInProgress)
        {
            // Start new rebuild
            _rebuildRowIndex = 0;
            _rebuildInProgress = true;
            _waterLevels.clear();

            // First pass: find all water levels
            std::unordered_set<MicroZ> foundLevels;
            for (int16_t y = 0; y < kMapRows; ++y)
            {
                for (int16_t x = 0; x < kMapColumns; ++x)
                {
                    auto tile = TileManager::get(TilePos2(x, y));
                    auto* surface = tile.surface();
                    if (surface != nullptr && surface->water() > 0)
                    {
                        foundLevels.insert(surface->water());
                    }
                }
            }

            // Initialize bitmaps for each level
            for (auto level : foundLevels)
            {
                auto& bitmap = _waterLevels[level];
                uint32_t maxDim = std::max(static_cast<uint32_t>(kMapRows), static_cast<uint32_t>(kMapColumns));
                bitmap.paddedSize = nextPowerOf2(maxDim);
                bitmap.numLevels = static_cast<uint32_t>(std::log2(bitmap.paddedSize)) + 1;
                uint32_t wordsPerRow = bitmap.paddedSize / 64;
                if (wordsPerRow == 0)
                    wordsPerRow = 1;
                bitmap.bits.assign(bitmap.paddedSize, std::vector<uint64_t>(wordsPerRow, 0));
                bitmap.dirty = true;
            }
        }

        // Process kRowsPerTick rows
        uint32_t endRow = std::min(_rebuildRowIndex + kRowsPerTick, static_cast<uint32_t>(kMapRows));
        for (uint32_t y = _rebuildRowIndex; y < endRow; ++y)
        {
            for (int16_t x = 0; x < kMapColumns; ++x)
            {
                auto tile = TileManager::get(TilePos2(x, static_cast<int16_t>(y)));
                auto* surface = tile.surface();
                if (surface != nullptr && surface->water() > 0)
                {
                    auto waterLevel = surface->water();
                    auto it = _waterLevels.find(waterLevel);
                    if (it != _waterLevels.end())
                    {
                        it->second.bits[y][x >> 6] |= (1ULL << (x & 63));
                    }
                }
            }
        }

        _rebuildRowIndex = endRow;

        // Check if rebuild is complete
        if (_rebuildRowIndex >= static_cast<uint32_t>(kMapRows))
        {
            // Build quadrant caches
            for (auto& [level, bitmap] : _waterLevels)
            {
                bitmap.buildQuadrantCache();
                bitmap.dirty = false;
            }

            _rebuildInProgress = false;
            _dirty = false;
            Diagnostics::Logging::info("WaterNetwork: Incremental rebuild complete, {} water levels", _waterLevels.size());
        }
    }

    WaterBitmap& WaterNetwork::getBitmapForLevel(MicroZ level)
    {
        auto it = _waterLevels.find(level);
        if (it != _waterLevels.end())
        {
            return it->second;
        }

        // Create new bitmap for this level
        _waterLevels[level] = WaterBitmap{};
        _waterLevels[level].build(level);
        _waterLevels[level].buildQuadrantCache();
        return _waterLevels[level];
    }

    bool WaterNetwork::isWater(TilePos2 pos, MicroZ waterLevel)
    {
        ensureInitialized();

        auto it = _waterLevels.find(waterLevel);
        if (it == _waterLevels.end())
        {
            return false;
        }

        return it->second.isWater(pos);
    }

    bool WaterNetwork::isTileWater(int32_t tx, int32_t ty, MicroZ waterLevel) const
    {
        if (tx < 0 || ty < 0 || tx >= kMapColumns || ty >= kMapRows)
            return false;

        auto it = _waterLevels.find(waterLevel);
        if (it == _waterLevels.end())
            return false;

        const auto& bitmap = it->second;
        if (static_cast<uint32_t>(ty) >= bitmap.bits.size())
            return false;

        uint32_t ux = static_cast<uint32_t>(tx);
        uint32_t uy = static_cast<uint32_t>(ty);

        if ((ux >> 6) >= bitmap.bits[uy].size())
            return false;

        return (bitmap.bits[uy][ux >> 6] >> (ux & 63)) & 1;
    }

    uint32_t WaterNetwork::getQuadrantWaterCount(MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy)
    {
        ensureInitialized();

        auto it = _waterLevels.find(waterLevel);
        if (it == _waterLevels.end())
            return 0;

        return it->second.getQuadrantWaterCount(level, qx, qy);
    }

    bool WaterNetwork::isPureWater(MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy)
    {
        auto it = _waterLevels.find(waterLevel);
        if (it == _waterLevels.end())
            return false;

        uint32_t tilesPerQuad = getTilesPerQuadrant(level);
        uint32_t maxTiles = tilesPerQuad * tilesPerQuad;
        return it->second.getQuadrantWaterCount(level, qx, qy) == maxTiles;
    }

    bool WaterNetwork::isAllLand(MicroZ waterLevel, uint32_t level, uint32_t qx, uint32_t qy)
    {
        return getQuadrantWaterCount(waterLevel, level, qx, qy) == 0;
    }

    WaterPathResult WaterNetwork::findPath(TilePos2 from, TilePos2 to, MicroZ waterLevel)
    {
        ensureInitialized();

        WaterPathResult result;
        result.hasPath = false;

        // Quick validation at tile level
        if (!isWater(from, waterLevel) || !isWater(to, waterLevel))
        {
            Diagnostics::Logging::warn("WaterNetwork: Start or end tile is not water!");
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

        Diagnostics::Logging::info("WaterNetwork: Pathfind from ({},{}) to ({},{}), distance={}",
                                   from.x, from.y, to.x, to.y, distance);

        // BFS at tile level
        struct TileNode
        {
            int16_t x, y;
            int16_t parentX, parentY;
            bool hasParent;
        };

        auto tileKey = [](int16_t x, int16_t y) -> uint32_t {
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

        static const int16_t dtx[] = {0, 1, 0, -1};
        static const int16_t dty[] = {-1, 0, 1, 0};

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

            for (int dir = 0; dir < 4; ++dir)
            {
                int16_t nx = current.x + dtx[dir];
                int16_t ny = current.y + dty[dir];

                uint32_t key = tileKey(nx, ny);
                if (visited.count(key) > 0)
                {
                    continue;
                }

                if (!isTileWater(nx, ny, waterLevel))
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
            Diagnostics::Logging::warn("WaterNetwork: NO PATH! Explored {} tiles", nodesExplored);
            return result;
        }

        Diagnostics::Logging::info("WaterNetwork: Path found! Explored {} tiles", nodesExplored);

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

        std::reverse(path.begin(), path.end());

        result.fullPath = path;

        // Simplify path: keep waypoints at intervals
        constexpr size_t kWaypointInterval = 8;
        for (size_t i = 1; i < path.size(); ++i)
        {
            if (i == path.size() - 1)
            {
                result.waypoints.push_back(path[i]);
            }
            else if (i % kWaypointInterval == 0)
            {
                result.waypoints.push_back(path[i]);
            }
        }

        if (result.waypoints.empty())
        {
            result.waypoints.push_back(to);
        }

        result.hasPath = true;

        Diagnostics::Logging::info("WaterNetwork: Simplified to {} waypoints", result.waypoints.size());

        return result;
    }

    std::vector<MicroZ> WaterNetwork::getWaterLevels() const
    {
        std::vector<MicroZ> levels;
        for (const auto& [level, bitmap] : _waterLevels)
        {
            levels.push_back(level);
        }
        return levels;
    }

    bool WaterNetwork::hasWaterLevel(MicroZ level) const
    {
        return _waterLevels.find(level) != _waterLevels.end();
    }

    uint32_t WaterNetwork::getPaddedSize() const
    {
        if (_waterLevels.empty())
            return 0;
        return _waterLevels.begin()->second.paddedSize;
    }

    uint32_t WaterNetwork::getNumLevels() const
    {
        if (_waterLevels.empty())
            return 0;
        return _waterLevels.begin()->second.numLevels;
    }

    uint32_t WaterNetwork::getNodeCount() const
    {
        uint32_t count = 0;
        for (const auto& [level, bitmap] : _waterLevels)
        {
            // Count water tiles across all levels
            for (const auto& row : bitmap.bits)
            {
                for (auto word : row)
                {
                    count += __builtin_popcountll(word);
                }
            }
        }
        return count;
    }
}
