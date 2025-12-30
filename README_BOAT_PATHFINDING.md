# Ship Pathfinding Implementation

This document describes the region-based pathfinding system for ships in OpenLoco.

## Overview

Ships navigate using a two-tier pathfinding approach:
1. **Region-level pathfinding** for long-distance navigation across large bodies of water
2. **Tile-level A*** for precise navigation in the final approach to destinations

This hybrid approach balances performance with accuracy, allowing ships to efficiently navigate complex water networks while avoiding obstacles near their destinations.

## Core Concepts

### Regions

**Regions** are contiguous areas of open water where simple greedy navigation works reliably. Key properties:

- Maximum size: 16×16 tiles (ensures convexity and prevents getting stuck)
- Minimum size: 10 tiles (smaller areas are filtered out)
- Each region has:
  - A unique ID
  - A water level (MicroZ)
  - A center point (for distance calculations)
  - A list of connected channels
  - A list of adjacent regions
  - All tiles within the region

### Channels

**Channels** are narrow water passages that connect regions. They represent constrained areas where ships must follow a specific path rather than navigate freely.

Properties:
- An ordered sequence of tiles forming the channel
- Water level
- Two connected regions (regionA and regionB)

### Adjacency

Regions can be **adjacent** without having a channel between them. This happens when regions are next to each other but separated by a single tile boundary. Adjacent regions can be navigated between directly without following a channel path.

### Docks and Waypoints

Docks and waypoints may not be located exactly on water tiles that belong to a region. The pathfinding system handles this by searching nearby tiles (up to 2 tiles away) to find the nearest water region. This connection is cached for performance.

## Implementation Details

### Network Detection (WaterWaypointNetwork.cpp)

The water navigation network is built in three phases:

#### Phase 1: Region Detection (`detectRegions`)

1. Scan all tiles in the game world
2. For each water tile, perform flood-fill to find contiguous water areas
3. Subdivide large water bodies into 16×16 cells to ensure convexity
4. Split cells containing land into separate sub-regions
5. Filter out regions smaller than 10 tiles
6. Store region data and build tile→region mapping

Key insight: Small, convex regions guarantee that greedy navigation (moving toward the goal) won't get stuck in local minima.

#### Phase 2: Channel Detection (`detectChannels`)

1. Identify narrow passages (tiles with exactly 2 water neighbors)
2. Trace channel paths through these passages
3. Connect channels to the regions they link
4. Store bidirectional channel information

#### Phase 3: Adjacent Region Detection (`detectAdjacentRegions`)

1. For each region, check tiles on its boundary
2. Look for neighboring regions in adjacent tiles
3. Create direct region-to-region connections
4. Store bidirectional adjacency relationships

### Region Lookup with Proximity Search

The `findRegionAt()` function implements smart region lookup:

```cpp
uint16_t findRegionAt(TilePos2 pos, MicroZ waterLevel)
```

1. First, check if the exact position is in `_tileToRegion` map
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
2. **Run A* between regions**: Navigate through the region graph using:
   - Channel lengths as edge costs (when available)
   - Manhattan distance between region centers (for adjacent regions)
3. **Build waypoint path**: Extract region centers as intermediate waypoints
4. **Return route**: Provide ordered list of positions to navigate through

The region-level pathfinding is cached per ship to avoid recalculating on every frame.

### Low-Level Navigation (VehicleHead.cpp)

The main pathfinding function integrates multiple navigation strategies:

#### Strategy 1: Waypoint Following (Long Distance)

When far from destination (>10 tiles):
1. Get cached waypoint path or recalculate if needed
2. Navigate toward the next waypoint using greedy movement
3. Use tile-level A* if stuck (few water directions available)
4. Advance to next waypoint when close enough

#### Strategy 2: Tile-Level A* (Final Approach)

When close to destination (≤10 tiles):
1. Try moving in all 4 directions
2. For each direction, run recursive A* to find best path to target
3. Select the direction with the best score
4. Move in that direction

This ensures ships can navigate around local obstacles and approach docks precisely.

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

**Critical separation of concerns**: Pathfinding guides ships to positions, but `updateWaterMotion()` handles the actual arrival detection and docking logic.

## Performance Optimizations

### Caching
- Region-level paths are cached per ship
- Tile→region lookups are cached when found via proximity search
- Network is only rebuilt when marked dirty

### Early Termination
- A* uses best-score tracking to prune search space
- Region pathfinding stops as soon as target is reached
- Tile A* has depth limits to prevent runaway recursion

### Lazy Initialization
- Network is only built when first needed
- `ensureInitialized()` checks dirty flag before rebuilding

### Metrics Tracking
- RIPF (Region/Island Pathfinding) calls tracked per frame
- Peak values logged when they expire
- Helps identify performance issues

## Key Files

- **WaterWaypointNetwork.h/cpp**: Network structure and detection
- **WaterWaypointPathfinding.h/cpp**: Region-level A* pathfinding
- **VehicleHead.cpp**: Integration and tile-level navigation
- **RoutingMetrics.h/cpp**: Performance tracking

## Design Decisions

### Why 16×16 Regions?
- Large enough to reduce number of regions (performance)
- Small enough to ensure convexity (correctness)
- Prevents greedy navigation from getting stuck in concave regions

### Why 10-Tile Minimum?
- Filters out tiny isolated water patches
- Reduces overhead from tracking too many small regions
- Small areas can be treated as channels instead

### Why Proximity Search for Docks?
- Docks are typically 1-2 tiles away from open water
- Avoids modifying network structure for each dock
- Automatically works for any waypoint type
- Caching prevents performance impact

### Why Separate Tile A* for Final Approach?
- Region centers may not align perfectly with dock positions
- Local obstacles (other ships, land features) need precise navigation
- Greedy movement can oscillate near the destination
- A* guarantees finding the shortest path when one exists

### Why Speed-Based Tolerance?
- Fast-moving ships need more stopping distance
- Prevents ships from overshooting docks at high speed
- Matches original game behavior
- Provides smooth docking experience across all speeds

## Future Improvements

Potential enhancements to consider:

1. **Dynamic Network Updates**: Rebuild only affected regions when water changes
2. **Multi-level Regions**: Hierarchy of region sizes for very large maps
3. **Traffic Awareness**: Factor in other ships when choosing paths
4. **Canal Lock Detection**: Special handling for water level transitions
5. **Performance Profiling**: Detailed metrics to identify bottlenecks
6. **Visualization Tools**: Debug rendering of regions, channels, and paths

## Debugging

Enable verbose logging to see pathfinding decisions:

```cpp
Diagnostics::Logging::info("Region path: {} -> {}", startRegion, targetRegion);
Diagnostics::Logging::info("Using tile A* for final approach");
```

Key things to watch for:
- "Ship or target not in a water region" - indicates dock not connecting
- "No waypoint path available" - may need to rebuild network
- Oscillating behavior - check tile A* vs greedy movement balance
- High RIPF values - may indicate excessive pathfinding calls

## Summary

This pathfinding system provides efficient, reliable ship navigation by:
1. Decomposing water into navigable regions
2. Using A* to find optimal routes through the region graph
3. Switching to tile-level precision near destinations
4. Automatically connecting docks via proximity search
5. Separating pathfinding from docking mechanics

The result is ships that can navigate complex water networks without getting stuck, while maintaining good performance even on large maps with many ships.
