#pragma once

#include <memory>
#include <string>

#include <algorithm>
#include "puppet/control/control_intent_types.hpp"
#include "puppet/primitive/primitive_types.hpp"
#include "puppet/runtime/runtime_config.hpp"
namespace puppet::retargeting {

    class RetargetingPlugin {
       public:
        virtual ~RetargetingPlugin() = default;

        virtual std::string name() const                                                 = 0;
        virtual bool configure(const runtime::RuntimeConfig& config, std::string& error) = 0;
        virtual bool requiresRobotState() const { return false; }

        virtual bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                             std::string& error) = 0;

        virtual bool bodyGroupValid(const std::string& input_bg) {
            if (bodyGroups_.empty()) {
                return true;
            }
            return std::find(bodyGroups_.begin(), bodyGroups_.end(), input_bg) != bodyGroups_.end();
        }

       protected:
        std::vector<std::string> bodyGroups_;  // 用于描述 当前 pluging 所支持处理的 bodyGroup有哪些
    };

    using RetargetingPluginPtr = std::shared_ptr<RetargetingPlugin>;

}  // namespace puppet::retargeting
