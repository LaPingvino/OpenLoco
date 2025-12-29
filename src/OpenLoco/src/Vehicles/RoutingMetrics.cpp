#include "RoutingMetrics.h"
#include "Config.h"
#include "Graphics/Gfx.h"
#include "Graphics/TextRenderer.h"
#include "Ui/WindowManager.h"
#include "VehicleManager.h"
#include "VehicleHead.h"
#include <OpenLoco/Diagnostics/Logging.h>

namespace OpenLoco::Vehicles::RoutingMetrics
{
    constexpr uint32_t kRollingAverageFrames = 60; // Average over 60 frames (~1.5 seconds at 40Hz)
    constexpr uint32_t kPeakHoldFrames = 400; // Hold peak for 10 seconds at 40Hz

    static uint32_t _callsThisFrame = 0;
    static uint32_t _ripfThisFrame = 0; // Relevant Instructions Per Frame
    static float _rollingAverageRIPF = 0.0f;
    static float _rollingAverageCalls = 0.0f;
    static float _peakRIPF = 0.0f;
    static float _peakCalls = 0.0f;
    static uint32_t _peakRIPFFramesRemaining = 0;
    static uint32_t _peakCallsFramesRemaining = 0;
    static uint32_t _frameCount = 0;

    void reset()
    {
        _callsThisFrame = 0;
        _ripfThisFrame = 0;
        _rollingAverageRIPF = 0.0f;
        _rollingAverageCalls = 0.0f;
        _peakRIPF = 0.0f;
        _peakCalls = 0.0f;
        _peakRIPFFramesRemaining = 0;
        _peakCallsFramesRemaining = 0;
        _frameCount = 0;
    }

    void recordWaterPathfindCall(uint32_t recursiveCallsMade)
    {
        _callsThisFrame++;
        _ripfThisFrame += recursiveCallsMade;
    }

    void frameEnd()
    {
        _frameCount++;
        
        // Update rolling averages using exponential moving average
        const float alpha = 2.0f / (kRollingAverageFrames + 1);
        _rollingAverageRIPF = _rollingAverageRIPF * (1.0f - alpha) + _ripfThisFrame * alpha;
        _rollingAverageCalls = _rollingAverageCalls * (1.0f - alpha) + _callsThisFrame * alpha;
        
        // Update peak values
        if (_rollingAverageRIPF > _peakRIPF)
        {
            _peakRIPF = _rollingAverageRIPF;
            _peakRIPFFramesRemaining = kPeakHoldFrames;
        }
        else if (_peakRIPFFramesRemaining > 0)
        {
            _peakRIPFFramesRemaining--;
            if (_peakRIPFFramesRemaining == 0)
            {
                // Log the expiring peak for optimization reference
                auto shipCount = getWaterVehicleCount();
                auto peakPerShip = shipCount > 0 ? _peakRIPF / static_cast<float>(shipCount) : 0.0f;
                Diagnostics::Logging::info("RoutingMetrics: Peak RIPF expired - RIPF: {:.0f}, Ships: {}, RIPF/ship: {:.1f}", 
                    _peakRIPF, shipCount, peakPerShip);
                _peakRIPF = _rollingAverageRIPF;
            }
        }
        
        if (_rollingAverageCalls > _peakCalls)
        {
            _peakCalls = _rollingAverageCalls;
            _peakCallsFramesRemaining = kPeakHoldFrames;
        }
        else if (_peakCallsFramesRemaining > 0)
        {
            _peakCallsFramesRemaining--;
            if (_peakCallsFramesRemaining == 0)
            {
                // Log the expiring peak for optimization reference
                Diagnostics::Logging::info("RoutingMetrics: Peak Calls/frame expired - {:.1f} calls/frame", _peakCalls);
                _peakCalls = _rollingAverageCalls;
            }
        }
        
        // Reset frame counters
        _callsThisFrame = 0;
        _ripfThisFrame = 0;
    }

    uint32_t getWaterVehicleCount()
    {
        uint32_t count = 0;
        for (auto* head : VehicleManager::VehicleList())
        {
            if (head->getTransportMode() == TransportMode::water)
            {
                count++;
            }
        }
        return count;
    }

    uint32_t getPathfindCallsThisFrame()
    {
        return _callsThisFrame;
    }

    uint32_t getRIPFThisFrame()
    {
        return _ripfThisFrame;
    }

    float getAverageRIPF()
    {
        return _rollingAverageRIPF;
    }

    float getRIPFPerShip()
    {
        auto shipCount = getWaterVehicleCount();
        if (shipCount == 0)
            return 0.0f;
        return _rollingAverageRIPF / static_cast<float>(shipCount);
    }

    void drawMetrics(Gfx::DrawingContext& drawingCtx)
    {
        auto shipCount = getWaterVehicleCount();
        auto avgRIPF = getAverageRIPF();
        auto peakRIPFPerShip = shipCount > 0 ? _peakRIPF / static_cast<float>(shipCount) : 0.0f;

        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Ships: %u | Calls/f: %.1f (pk: %.1f) | RIPF: %.0f (pk: %.0f, %.1f/ship)", 
                 shipCount, _rollingAverageCalls, _peakCalls, avgRIPF, _peakRIPF, peakRIPFPerShip);

        auto tr = Gfx::TextRenderer(drawingCtx);
        
        auto stringWidth = tr.getStringWidthNewLined(buffer);
        
        // Draw at top-center, below FPS counter if it's visible
        int16_t y = 2;
        if (Config::get().showFPS)
        {
            y = 15; // Draw below FPS counter
        }
        
        int16_t x = Ui::width() / 2 - (stringWidth / 2);
        
        auto point = Ui::Point(x, y);
        
        tr.setCurrentFont(Gfx::Font::medium_bold);
        tr.drawString(point, AdvancedColour(Colour::white).outline(), buffer);
        
        // Invalidate the region so it gets redrawn
        Gfx::invalidateRegion(x, y, x + stringWidth, y + 14);
    }
}
