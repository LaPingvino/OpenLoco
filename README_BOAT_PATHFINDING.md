# Ship Pathfinding Implementation

This document describes the two-level hierarchical pathfinding system for ships in OpenLoco.

## Overview

Ships navigate using a clean two-level A* pathfinding approach:
1. **Region-level A*** - Strategic pathfinding across the water network
2. **Tile-level A*** - Tactical pathfinding to execute the route

This hierarchical approach balances performance with reliability, allowing ships to efficiently navigate complex water networks while guaranteeing they reach their destinations when a path exists.

## Core Concepts

### Regions

**Regions** are contiguous areas of open water identified by the pathfinding system. Key properties:

- Maximum size: 16×16 tiles (ensures manageable pathfinding complexity)
- Minimum size: 10 tiles (smaller areas are filtered out)
- Each region has:
  - A unique ID
  - A water level (MicroZ)
  - A center point (used as waypoint for navigation)
  - A list of adjacent regions
  - All tiles within the region

**Why regions?** They abstract large water bodies into a navigation graph, reducing the pathfinding search space from thousands of tiles to hundreds of regions.

### Adjacency

Regions can be **adjacent** when they share a border. Adjacent regions are directly connected in the navigation graph without needing intermediate structures.

### Docks and Waypoints

Docks and waypoints may not be located exactly on water tiles that belong to a region. The pathfinding system handles this by searching nearby tiles (up to 2 tiles away) to find the nearest water region. This connection is cached for performance.

## Two-Level Pathfinding Architecture

### Level 1: Region-Level A* (Strategic)

The high-level pathfinding layer that plans the route:

**Input:** Start position and destination position

**Process:**
1. Find which region contains the ship's current position
2. Find which region contains the destination (using proximity search for docks)
3. Run A* through the region graph using:
   - Manhattan distance between region centers as heuristic
   - Number of region transitions as cost
4. Extract region centers as waypoints

**Output:** Ordered list of waypoints to visit (one per region on the path)

**Caching:** Results are cached per ship to avoid recalculating every frame

### Level 2: Tile-Level A* (Tactical)

The low-level pathfinding layer that executes the route:

**Input:** Current position and next waypoint (or final destination)

**Process:**
1. For each of 4 cardinal directions from current position
2. Run recursive A* pathfinding to the target
3. Select the direction with the best score
4. Prefer current direction when scores are equal (momentum)

**Output:** Next position to move to

**When used:**
- To reach each waypoint from the region path
- For final approach to destination (≤10 tiles)
- When no region path exists (fallback to direct tile A*)

## Implementation Details

### Network Detection (WaterWaypointNetwork.cpp)

The water navigation network is built in three phases:

#### Phase 1: Region Detection (`detectRegions`)

1. Sample water tiles across the map at regular intervals
2. For each unvisited water tile, perform flood-fill to find contiguous water
3. Subdivide large water bodies into 16×16 cells
4. Split cells containing land into separate sub-regions
5. Filter out regions smaller than 10 tiles
6. Assign sequential region IDs and build tile→region mapping

**Key insight:** Small regions with maximum size guarantee bounded pathfinding complexity while still covering the entire water network.

#### Phase 2: Channel Detection (`detectChannels`)

Currently unused - channels were part of an earlier design but are no longer necessary with the two-level A* approach.

#### Phase 3: Adjacent Region Detection (`detectAdjacentRegions`)

1. For each region, examine tiles on its boundary
2. Check neighboring tiles for different regions
3. Create bidirectional adjacency connections
4. Build the region navigation graph

### Region Lookup with Proximity Search

The `findRegionAt()` function implements smart region lookup:

```cpp
uint16_t findRegionAt(TilePos2 pos, MicroZ waterLevel)
```

1. Check if the exact position is in `_tileToRegion` map
2. If not found, search nearby tiles in expanding pattern:
   - Adjacent tiles (4 directions)
   - Diagonal tiles (4 directions)  
   - Tiles 2 steps away (4 directions)
3. When a nearby region is found, cache the result for future lookups
4. Return the region ID or 0xFFFF if no region found

This approach allows docks, waypoints, and other non-water positions to automatically connect to the nearest water region without requiring special handling.

### High-Level Pathfinding (WaterWaypointPathfinding.cpp)

When a ship needs to navigate to a distant destination:

1. **Find start and target regions**: Use `findRegionAt()` for both current position and destination
2. **Run A* between regions**: Navigate through the region graph using Manhattan distance between region centers as both cost and heuristic
3. **Build waypoint path**: Extract region centers as intermediate waypoints
4. **Cache the result**: Store the path in `_cachedRegionPaths` map indexed by ship ID

**Path invalidation:** The cache is invalidated when:
- Target position changes
- Ship gets stuck (same position for too many frames)

**Special case:** If start and target are in the same region, return the target directly (no waypoints needed).

### Low-Level Navigation (VehicleHead.cpp)

The main pathfinding function integrates the two levels:

#### Strategy 1: Waypoint Following (Long Distance)

When far from destination (>10 tiles) and a region path exists:
1. Get the next waypoint from the cached path
2. Skip waypoints that are too close (already reached)
3. Use tile-level A* to navigate to the waypoint
4. When waypoint is reached, advance to the next one

#### Strategy 2: Tile-Level A* (Final Approach)

When close to destination (≤10 tiles):
1. Run tile A* in all 4 directions
2. Select the direction with the best pathfinding result
3. Move in that direction

This ensures ships can precisely approach docks and navigate around local obstacles.

#### Strategy 3: Dock Handoff (Very Close)

When within 2 tiles of a dock target:
1. Calculate world-space distance (not just tile distance)
2. Check against tolerance based on ship speed:
   - 3 units at low speed (<20 mph)
   - 16 units at medium speed (20-70 mph)
   - 24 units at high speed (>70 mph)
3. If within tolerance + 16 unit margin, return dock target directly
4. This triggers the handoff to `updateWaterMotion()` for final docking

### Docking Detection (updateWaterMotion)

The actual dock arrival is detected in `updateWaterMotion()`, NOT in the pathfinding code:

1. Calculate `manhattanDistance2D(head.position, veh2->position)`
2. Compare against speed-based tolerance
3. When `manhattanDistance <= targetTolerance`:
   - Set `hasReachedADestination` flag
   - Set `hasReachedDock` flag if stationId is valid
4. Main update loop checks these flags and triggers docking sequence

**Critical separation of concerns:** Pathfinding guides ships to positions, `updateWaterMotion()` handles the actual arrival detection and docking logic.

## Performance Optimizations

### Caching
- Region-level paths are cached per ship
- Tile→region lookups are cached when found via proximity search
- Network is only rebuilt when marked dirty

### Lazy Initialization
- Network is only built when first needed
- `ensureInitialized()` checks dirty flag before rebuilding

### Bounded Complexity
- Region A* operates on ~280 regions instead of ~10000+ tiles
- Tile A* is only used for short distances (waypoint-to-waypoint or final approach)
- Maximum region size (16×16) bounds the tile A* search space

### Early Termination
- A* uses best-score tracking to prune search space
- Pathfinding stops as soon as target is reached

## Key Files

- **WaterWaypointNetwork.h/cpp**: Network structure, region detection, and adjacency
- **WaterWaypointPathfinding.h/cpp**: Region-level A* pathfinding
- **VehicleHead.cpp**: Two-level pathfinding integration and tile-level A*
- **RoutingMetrics.h/cpp**: Performance tracking (console logging only)
- **Scenario.cpp**: Network reset on game load/unload

## Design Decisions

### Why Two Levels of A*?

**Region-level A*:** 
- Reduces search space from thousands of tiles to hundreds of regions
- Provides strategic routing across large water bodies
- Cached to avoid recalculation every frame

**Tile-level A*:**
- Guarantees finding optimal path when one exists
- Handles local obstacles and precise navigation
- Only used for short distances (bounded complexity)

**Why not greedy movement?** Greedy movement can get stuck in local minima, oscillate between positions, and fail to find paths around obstacles. Tile A* is more reliable and still performant for short distances.

### Why 16×16 Regions?
- Large enough to reduce the number of regions (fewer nodes in graph)
- Small enough to keep tile A* bounded when navigating within regions
- Balances pathfinding performance with path quality

### Why 10-Tile Minimum?
- Filters out tiny isolated water patches that don't contribute to navigation
- Reduces graph complexity without losing important water areas
- Small areas are implicitly handled by proximity search

### Why Proximity Search for Docks?
- Docks are typically 1-2 tiles away from open water
- Avoids modifying network structure for each dock
- Automatically works for any waypoint type
- Caching prevents performance impact

### Why Speed-Based Tolerance?
- Fast-moving ships need more stopping distance
- Prevents ships from overshooting docks at high speed
- Matches original game behavior
- Provides smooth docking experience across all speeds

### Why Prefer Current Direction?
- Adds momentum to ship movement
- Prevents oscillation when multiple directions have equal cost
- Results in smoother, more natural navigation

## Network Lifecycle

### Initialization
1. Triggered on first pathfinding request via `ensureInitialized()`
2. Scans map and builds region graph
3. Marks network as initialized

### Updates
- Network is marked dirty when water terrain changes
- Automatically rebuilds on next pathfinding request

### Reset
- Called when loading a new game or scenario
- Clears all regions, adjacency data, and tile mappings
- Prevents stale data from persisting between games

## Debugging

All pathfinding decisions are logged to the console for debugging:

**Region pathfinding:**
```
WaterWaypointPathfinding: Region A* from region 160 to 197 (water level 4)
WaterWaypointPathfinding: Path found! 19 regions, 0 channels, 19 tiles
```

**Waypoint following:**
```
V0 [9] (0): Following waypoint at (56,360)
Tile A* to waypoint succeeded - direction 2
```

**Cache behavior:**
```
V0 [9] (0): CACHE MISS - Recalculating region path
V0 [9] (0): CACHE HIT - Using cached path (routeIdx=3)
```

**Stuck detection:**
```
V0 [9] (0): Stuck in same position for 30 frames - invalidating path
```

Key things to watch for:
- "Ship or target not in a water region" - dock proximity search may have failed
- "No region path found" - no route exists between start and destination
- Frequent cache misses - ship may be stuck or path is being invalidated often
- Tile A* failures - local obstacles may be blocking waypoint navigation

## Algorithm Comparison

### Previous Approach (Greedy Movement)
```
For each direction:
  Calculate Manhattan distance to waypoint
  Pick direction with minimum distance
```
**Problems:** 
- Gets stuck in concave areas
- Oscillates when multiple directions have similar distances
- No guarantee of reaching the waypoint

### Current Approach (Tile-Level A*)
```
For each direction:
  Run A* pathfinding to waypoint
  Evaluate complete path quality
  Pick direction with best A* result
```
**Benefits:**
- Guaranteed to find path if one exists
- Handles obstacles and complex geometry
- No oscillation or getting stuck
- Minimal performance impact (bounded search)

## Future Improvements

Potential enhancements to consider:

1. **Dynamic Network Updates**: Rebuild only affected regions when water changes
2. **Multi-level Regions**: Hierarchy of region sizes for very large maps
3. **Traffic Awareness**: Consider other ships when choosing paths
4. **Path Smoothing**: Post-process waypoint paths to reduce unnecessary turns
5. **Bidirectional Search**: Speed up tile A* by searching from both ends
6. **Jump Point Search**: Optimize tile A* for open water areas

## Summary

This pathfinding system provides efficient, reliable ship navigation through a clean two-level architecture:

1. **Region-level A*** finds the strategic route through the water network
2. **Tile-level A*** executes that route with guaranteed waypoint arrival
3. **Proximity search** seamlessly connects docks to nearby water regions
4. **Automatic reset** prevents data from persisting between games

The result is ships that navigate complex water networks reliably without getting stuck, while maintaining good performance even on large maps with many ships. The two-level approach separates strategic planning from tactical execution, making the system both efficient and maintainable.
