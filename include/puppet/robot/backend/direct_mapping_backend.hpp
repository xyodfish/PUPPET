#pragma once

#include "puppet/control/control_intent_types.hpp"
#include "puppet/target/target_types.hpp"

namespace puppet::robot {

    class DirectMappingBackend {
       public:
        target::FinalTarget buildTarget(const model::ControlIntent& controlIntent) const;
    };

}  // namespace puppet::robot
