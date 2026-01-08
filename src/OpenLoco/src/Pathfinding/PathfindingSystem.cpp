#include "PathfindingSystem.h"
#include "WaterNetwork.h"
#include "TrackNetwork.h"
#include "RoadNetwork.h"
#include <OpenLoco/Diagnostics/Logging.h>

namespace OpenLoco::Pathfinding
{
    // Static network instances
    static WaterNetwork _waterNetwork;
    static TrackNetwork _trackNetwork;
    static RoadNetwork _roadNetwork;

    // Track which modes have active pathfinding
    static uint32_t _activePathfindingModes = 0;

    namespace PathfindingSystem
    {
        void initialize()
        {
            Diagnostics::Logging::info("PathfindingSystem: Initializing...");

            _waterNetwork.initialize();
            _trackNetwork.initialize();
            _roadNetwork.initialize();

            _activePathfindingModes = 0;

            Diagnostics::Logging::info("PathfindingSystem: Initialization complete");
        }

        void reset()
        {
            Diagnostics::Logging::info("PathfindingSystem: Resetting...");

            _waterNetwork.reset();
            _trackNetwork.reset();
            _roadNetwork.reset();

            _activePathfindingModes = 0;
        }

        void updatePeriodicRefresh()
        {
            _waterNetwork.updatePeriodicRefresh();
            _trackNetwork.updatePeriodicRefresh();
            _roadNetwork.updatePeriodicRefresh();
        }

        WaterNetwork& getWaterNetwork()
        {
            return _waterNetwork;
        }

        TrackNetwork& getTrackNetwork()
        {
            return _trackNetwork;
        }

        RoadNetwork& getRoadNetwork()
        {
            return _roadNetwork;
        }

        void markWaterDirty()
        {
            _waterNetwork.markDirty();
        }

        void markTrackDirty([[maybe_unused]] World::Pos3 pos, [[maybe_unused]] uint8_t trackObjectId)
        {
            // For now, mark entire network dirty
            // Future optimization: only invalidate affected region
            _trackNetwork.markDirty();
        }

        void markRoadDirty([[maybe_unused]] World::Pos3 pos, [[maybe_unused]] uint8_t roadObjectId)
        {
            // For now, mark entire network dirty
            // Future optimization: only invalidate affected region
            _roadNetwork.markDirty();
        }

        void markAllDirty()
        {
            _waterNetwork.markDirty();
            _trackNetwork.markDirty();
            _roadNetwork.markDirty();
        }

        bool hasActivePathfinding(TransportMode mode)
        {
            return (_activePathfindingModes & (1U << static_cast<uint8_t>(mode))) != 0;
        }

        uint32_t getActiveTransportModeFlags()
        {
            return _activePathfindingModes;
        }

        std::vector<TransportMode> getActiveTransportModes()
        {
            std::vector<TransportMode> modes;
            for (uint8_t i = 0; i < static_cast<uint8_t>(TransportMode::Count); ++i)
            {
                if (_activePathfindingModes & (1U << i))
                {
                    modes.push_back(static_cast<TransportMode>(i));
                }
            }
            return modes;
        }

        uint32_t getWaterNodeCount()
        {
            return _waterNetwork.getNodeCount();
        }

        uint32_t getTrackNodeCount()
        {
            return _trackNetwork.getNodeCount();
        }

        uint32_t getRoadNodeCount()
        {
            return _roadNetwork.getNodeCount();
        }

        // Functions to update active pathfinding state (called by networks)
        void setTransportModeActive(TransportMode mode, bool active)
        {
            if (active)
            {
                _activePathfindingModes |= (1U << static_cast<uint8_t>(mode));
            }
            else
            {
                _activePathfindingModes &= ~(1U << static_cast<uint8_t>(mode));
            }
        }
    }
}
