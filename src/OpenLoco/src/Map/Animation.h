#pragma once

#include <OpenLoco/Engine/World.hpp>

namespace OpenLoco::World
{
#pragma pack(push, 1)
    struct Animation
    {
        uint8_t baseZ;
        uint8_t type;
        World::Pos2 pos;
    };
    // static_assert(sizeof(Animation) == 6); // COMMENTED FOR 64-BIT DEBUG
#pragma pack(pop)
}
