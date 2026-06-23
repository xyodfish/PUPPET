#pragma once
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "puppet/retargeting/core/retargeting_plugin.hpp"

namespace puppet::retargeting {
    class KCRetargetingPlugin final : public RetargetingPlugin {
       public:
        bool bodyGroupValid(const std::string& input_bg) override;
        bool configure(const runtime::RuntimeConfig& config, std::string& error) override;
        std::string name() const override;
        bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                     std::string& error) override;
    };
}  // namespace puppet::retargeting