#pragma once

#include <memory>
#include <string>

#include "puppet/control/control_intent_types.hpp"
#include "puppet/primitive/primitive_types.hpp"
#include "puppet/runtime/runtime_config.hpp"

namespace puppet::retargeting {

    class RetargetingPlugin {
       public:
        virtual ~RetargetingPlugin() = default;

        virtual std::string name() const                                                 = 0;
        virtual bool configure(const runtime::RuntimeConfig& config, std::string* error) = 0;

        virtual bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                             std::string* error) = 0;
    };

    using RetargetingPluginPtr = std::shared_ptr<RetargetingPlugin>;

}  // namespace puppet::retargeting
