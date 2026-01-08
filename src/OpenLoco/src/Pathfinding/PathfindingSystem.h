#pragma once

#include <OpenLoco/Engine/World.hpp>
#include <cstdint>
#include <vector>

namespace OpenLoco::Pathfinding
{
    // Forward declarations
    class WaterNetwork;
    class TrackNetwork;
    class RoadNetwork;

    enum class TransportMode : uint8_t
    {
        Water = 0,
        Rail = 1,
        Road = 2,
        Tram = 3,
        Aircraft = 4, // For visualization only (no pathfinding needed)
        Count = 5
    };

    // Global pathfinding system managing all network types
    namespace PathfindingSystem
    {
        // Lifecycle management
        void initialize();
        void reset();
        void updatePeriodicRefresh();

        // Network accessors
        WaterNetwork& getWaterNetwork();
        TrackNetwork& getTrackNetwork();
        RoadNetwork& getRoadNetwork();

        // Invalidation - call when infrastructure changes
        void markWaterDirty();
        void markTrackDirty(World::Pos3 pos, uint8_t trackObjectId);
        void markRoadDirty(World::Pos3 pos, uint8_t roadObjectId);
        void markAllDirty();

        // Debug info for minimap visualization
        bool hasActivePathfinding(TransportMode mode);
        uint32_t getActiveTransportModeFlags(); // Bitmask of active modes
        std::vector<TransportMode> getActiveTransportModes();
        void setTransportModeActive(TransportMode mode, bool active);

        // Statistics
        uint32_t getWaterNodeCount();
        uint32_t getTrackNodeCount();
        uint32_t getRoadNodeCount();
    }
}
