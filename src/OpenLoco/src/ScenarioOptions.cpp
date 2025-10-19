#include "ScenarioOptions.h"
#include <OpenLoco/Interop/Interop.hpp>

namespace OpenLoco::Scenario
{
    static Options _activeOptions = {}; // Was loco_global at 0x009C8714

    Options& getOptions()
    {
        return _activeOptions;
    }
}
