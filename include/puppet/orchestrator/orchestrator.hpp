#pragma once

#include <string>
#include <vector>

#include "puppet/runtime/runtime_config.hpp"

namespace puppet::orchestrator {

    struct GroupExecutionPlan {
        std::string bodyGroup;
        std::string ownerSourceId;
        std::string pipelineId;
        std::string backendId;
        std::string mode;
        std::string controlSemantics;
        int32_t priority = 0;
    };

    class Orchestrator {
       public:
        void configure(const std::vector<runtime::GroupRoutingConfig>& groupRouting);
        std::vector<GroupExecutionPlan> resolvePlans() const;

       private:
        std::vector<runtime::GroupRoutingConfig> groupRouting_;
    };

}  // namespace puppet::orchestrator
