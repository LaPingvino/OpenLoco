#pragma once

#include <cstdint>

namespace OpenLoco::Gfx
{
    class DrawingContext;
}

namespace OpenLoco::Vehicles::RoutingMetrics
{
    void reset();
    void recordWaterPathfindCall(uint32_t recursiveCallsMade); // Track RIPF (Relevant Instructions Per Frame)
    void frameEnd(); // Called at end of each frame to update rolling averages
    uint32_t getWaterVehicleCount();
    uint32_t getPathfindCallsThisFrame();
    uint32_t getRIPFThisFrame(); // Relevant Instructions Per Frame
    float getAverageRIPF(); // Rolling average RIPF
    float getRIPFPerShip(); // RIPF normalized per ship
    void drawMetrics(Gfx::DrawingContext& drawingCtx);
}
