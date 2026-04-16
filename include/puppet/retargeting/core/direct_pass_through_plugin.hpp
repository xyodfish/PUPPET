#pragma once

#include "puppet/retargeting/core/retargeting_plugin.hpp"

namespace puppet::retargeting {

    class DirectPassThroughPlugin final : public RetargetingPlugin {
       public:
        std::string name() const override { return "direct_pass"; }
        bool configure(const runtime::RuntimeConfig& config, std::string* error) override;
        bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                     std::string* error) override;
    };

}  // namespace puppet::retargeting
