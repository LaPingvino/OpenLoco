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
        static void buildQuadrantCache()
        {
            // Allocate cache for all levels
            _quadrantCache.resize(_numLevels);

            // Start from finest level (2x2 tile quadrants)
            uint32_t finestLevel = _numLevels - 1;
            uint32_t quadrantsPerSide = _paddedSize / 2; // At finest level, each quadrant is 2x2 tiles

            // Level numLevels-1: count water in each 2x2 tile block
            _quadrantCache[finestLevel].assign(quadrantsPerSide, std::vector<uint32_t>(quadrantsPerSide, 0));

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
                    _quadrantCache[finestLevel][qy][qx] = count;
                }
            }

            // Build coarser levels bottom-up
            // Each coarser level quadrant = sum of 4 child quadrants
            for (int32_t level = static_cast<int32_t>(finestLevel) - 1; level >= 0; --level)
            {
                uint32_t numQuadrants = 1U << level; // 2^level quadrants per side at this level
                _quadrantCache[level].assign(numQuadrants, std::vector<uint32_t>(numQuadrants, 0));

                for (uint32_t qy = 0; qy < numQuadrants; ++qy)
                {
                    for (uint32_t qx = 0; qx < numQuadrants; ++qx)
                    {
                        // Sum 4 children from level+1
                        uint32_t childX = qx * 2;
                        uint32_t childY = qy * 2;
                        uint32_t childLevel = level + 1;

                        uint32_t sum = 0;
                        sum += _quadrantCache[childLevel][childY][childX];
                        sum += _quadrantCache[childLevel][childY][childX + 1];
                        sum += _quadrantCache[childLevel][childY + 1][childX];
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
            // Level 0 = whole map = _paddedSize tiles
            // Level numLevels-1 = 2 tiles per quadrant
            return _paddedSize >> level;
        }

        // Convert tile position to quadrant coordinates at given level
        static void tileToQuadrant(TilePos2 pos, uint32_t level, uint32_t& qx, uint32_t& qy)
        {
            uint32_t tilesPerQuad = getTilesPerQuadrant(level);
            qx = static_cast<uint32_t>(pos.x) / tilesPerQuad;
            qy = static_cast<uint32_t>(pos.y) / tilesPerQuad;
        }

        // Get center tile of a quadrant
        static TilePos2 getQuadrantCenter(uint32_t level, uint32_t qx, uint32_t qy)
        {
            uint32_t tilesPerQuad = getTilesPerQuadrant(level);
            int16_t cx = static_cast<int16_t>(qx * tilesPerQuad + tilesPerQuad / 2);
            int16_t cy = static_cast<int16_t>(qy * tilesPerQuad + tilesPerQuad / 2);
            return TilePos2(cx, cy);
        }
    }

    void initialize()
    {
        Diagnostics::Logging::info("WaterBinaryMap: Initializing...");
        buildBitmap();
        buildQuadrantCache();
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

    void reset()
    {
        Diagnostics::Logging::info("WaterBinaryMap: Resetting");
        _waterBits.clear();
        _quadrantCache.clear();
        _paddedSize = 0;
        _numLevels = 0;
        _initialized = false;
        _dirty = true;
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

        uint32_t numQuadrants = 1U << level;
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

    // Hierarchical pathfinding using quadrant BFS
    PathResult findPath(TilePos2 from, TilePos2 to, MicroZ waterLevel)
    {
        ensureInitialized();

        PathResult result;
        result.hasPath = false;

        // Quick validation
        if (!isWater(from, waterLevel) || !isWater(to, waterLevel))
        {
            return result;
        }

        // If very close, no waypoints needed
        int32_t dx = std::abs(from.x - to.x);
        int32_t dy = std::abs(from.y - to.y);
        if (dx <= 8 && dy <= 8)
        {
            result.hasPath = true;
            result.waypoints.push_back(to);
            return result;
        }

        // Choose pathfinding level based on distance
        // For long distances, use coarser quadrants (faster)
        // For shorter distances, use finer quadrants (more precise)
        uint32_t pathLevel;
        int32_t distance = dx + dy;
        if (distance > 128)
        {
            pathLevel = std::min(4U, _numLevels - 1); // 32x32 tile quadrants
        }
        else if (distance > 32)
        {
            pathLevel = std::min(5U, _numLevels - 1); // 16x16 tile quadrants
        }
        else
        {
            pathLevel = std::min(6U, _numLevels - 1); // 8x8 tile quadrants
        }

        uint32_t numQuadrants = 1U << pathLevel;

        // Get start and end quadrants
        uint32_t startQx, startQy, endQx, endQy;
        tileToQuadrant(from, pathLevel, startQx, startQy);
        tileToQuadrant(to, pathLevel, endQx, endQy);

        // BFS on quadrant graph
        struct QuadNode
        {
            uint32_t qx, qy;
            uint32_t parentQx, parentQy;
            bool hasParent;
        };

        auto quadKey = [numQuadrants](uint32_t qx, uint32_t qy) -> uint64_t
        {
            return (static_cast<uint64_t>(qy) << 32) | qx;
        };

        std::queue<QuadNode> openSet;
        std::unordered_set<uint64_t> visited;
        std::vector<QuadNode> allNodes;

        QuadNode startNode = {startQx, startQy, 0, 0, false};
        openSet.push(startNode);
        allNodes.push_back(startNode);
        visited.insert(quadKey(startQx, startQy));

        bool found = false;
        size_t endNodeIdx = 0;

        // Direction offsets for 4-connectivity
        static const int32_t dqx[] = {0, 1, 0, -1};
        static const int32_t dqy[] = {-1, 0, 1, 0};

        while (!openSet.empty() && !found)
        {
            QuadNode current = openSet.front();
            openSet.pop();

            if (current.qx == endQx && current.qy == endQy)
            {
                found = true;
                // Find this node in allNodes
                for (size_t i = 0; i < allNodes.size(); ++i)
                {
                    if (allNodes[i].qx == current.qx && allNodes[i].qy == current.qy)
                    {
                        endNodeIdx = i;
                        break;
                    }
                }
                break;
            }

            // Explore neighbors
            for (int dir = 0; dir < 4; ++dir)
            {
                int32_t nqx = static_cast<int32_t>(current.qx) + dqx[dir];
                int32_t nqy = static_cast<int32_t>(current.qy) + dqy[dir];

                if (nqx < 0 || nqy < 0 ||
                    static_cast<uint32_t>(nqx) >= numQuadrants ||
                    static_cast<uint32_t>(nqy) >= numQuadrants)
                {
                    continue;
                }

                uint64_t key = quadKey(nqx, nqy);
                if (visited.count(key) > 0)
                {
                    continue;
                }

                // Skip all-land quadrants
                if (isAllLand(pathLevel, nqx, nqy))
                {
                    continue;
                }

                visited.insert(key);
                QuadNode neighbor = {static_cast<uint32_t>(nqx), static_cast<uint32_t>(nqy),
                                     current.qx, current.qy, true};
                openSet.push(neighbor);
                allNodes.push_back(neighbor);
            }
        }

        if (!found)
        {
            Diagnostics::Logging::verbose("WaterBinaryMap: No quadrant path from ({},{}) to ({},{})",
                                          from.x, from.y, to.x, to.y);
            return result;
        }

        // Reconstruct path through quadrants
        std::vector<TilePos2> quadrantCenters;
        size_t nodeIdx = endNodeIdx;

        while (true)
        {
            QuadNode& node = allNodes[nodeIdx];
            quadrantCenters.push_back(getQuadrantCenter(pathLevel, node.qx, node.qy));

            if (!node.hasParent)
            {
                break;
            }

            // Find parent node
            bool foundParent = false;
            for (size_t i = 0; i < allNodes.size(); ++i)
            {
                if (allNodes[i].qx == node.parentQx && allNodes[i].qy == node.parentQy)
                {
                    nodeIdx = i;
                    foundParent = true;
                    break;
                }
            }
            if (!foundParent)
                break;
        }

        // Reverse to get path from start to end
        std::reverse(quadrantCenters.begin(), quadrantCenters.end());

        // Build waypoints: skip first (we're already there), add intermediate centers, end with target
        for (size_t i = 1; i < quadrantCenters.size(); ++i)
        {
            result.waypoints.push_back(quadrantCenters[i]);
        }
        result.waypoints.push_back(to);

        result.hasPath = true;

        Diagnostics::Logging::verbose("WaterBinaryMap: Path found with {} waypoints", result.waypoints.size());

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
}
