#pragma once

#include <cstdint>

namespace OpenLoco::Vehicles
{
    // RIPF = Relevant Instructions Per Frame
    // Used to measure computational cost independent of FPS
    namespace RoutingMetrics
    {
        struct RIPFTracker
        {
            RIPFTracker();
            ~RIPFTracker();
        };

        // Track RIPF cost for a scope
        RIPFTracker trackRIPF();
        
        // Get total RIPF cost accumulated this frame
        uint64_t getTotalRIPF();
        
        // Reset counters (call at start of frame)
        void resetFrame();
    }
}
