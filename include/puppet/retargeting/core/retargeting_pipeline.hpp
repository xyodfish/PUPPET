#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "puppet/control/control_intent_types.hpp"
#include "puppet/primitive/primitive_types.hpp"
#include "puppet/retargeting/core/direct_pass_through_plugin.hpp"
#include "puppet/retargeting/core/retargeting_plugin.hpp"
#include "puppet/runtime/runtime_config.hpp"

namespace puppet::retargeting {

    class RetargetingPipeline {
       public:
        bool configure(const runtime::RuntimeConfig& config, std::string* error);

        bool run(const std::string& pipelineId, const model::PrimitiveFrame& frame, const std::string& bodyGroup,
                 model::GroupControlIntent* output, std::string* error) const;

       private:
        std::unordered_map<std::string, RetargetingPluginPtr> plugins_;
    };

}  // namespace puppet::retargeting
