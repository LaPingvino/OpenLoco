#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <vector>
#include <cstdint>

namespace OpenLoco::Vehicles
{
    // A channel is a narrow passage that must be followed carefully
    struct WaterChannel
    {
        std::vector<World::TilePos2> path; // Ordered tiles forming the channel
        World::MicroZ waterLevel;
        uint16_t regionA; // Region at one end
        uint16_t regionB; // Region at other end
    };

    // A region is open water where greedy navigation works
    // Regions are kept small and convex to guarantee greedy pathfinding
    struct WaterRegion
    {
        uint16_t regionId;
        World::MicroZ waterLevel;
        World::TilePos2 centerPoint; // Approximate center for distance calculations
        std::vector<uint16_t> connectedChannels; // Channels leading out of this region
        std::vector<uint16_t> connectedRegions; // Regions reachable via channels
        
        // Bounding box for quick checks (regions are subdivided to keep this small)
        int16_t minX, maxX, minY, maxY;
        
        // All tiles in this region (for convexity checking)
        std::vector<World::TilePos2> tiles;
    };

    namespace WaterWaypointNetwork
    {
        void initialize();
        void ensureInitialized();
        void markDirty();
        void reset(); // Clear all data (call on game unload)
        
        // Find which region a position is in
        uint16_t findRegionAt(World::TilePos2 pos, World::MicroZ waterLevel);
        
        // Get region info
        const WaterRegion* getRegion(uint16_t regionId);
        const std::vector<WaterRegion>& getAllRegions();
        
        // Get channel info
        const WaterChannel* getChannel(uint16_t channelId);
        const std::vector<WaterChannel>& getAllChannels();
    }
}
