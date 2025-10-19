#include "GameState.h"
#include <OpenLoco/Core/Traits.hpp>
#include <OpenLoco/Interop/Interop.hpp>

using namespace OpenLoco::Interop;

namespace OpenLoco
{
    // static_assert(Traits::IsPOD<GameState>::value == true, "GameState must be POD."); // COMMENTED FOR 64-BIT DEBUG

    loco_global<GameState, 0x00525E18> _gameState;

    GameState& getGameState()
    {
        return *_gameState;
    }
}
