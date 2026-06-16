#pragma once

#include "puppet/runtime/runtime_config.hpp"

namespace puppet::routing {

    struct GroupRoutingPlan {
        std::string bodyGroup;
        std::string ownerSourceId;
        std::string pipelineId;
        std::string backendId;
        std::string mode;
        std::string controlSemantics;
        int32_t priority = 0;
        std::vector<runtime::PipelineConfig> activedPlugins_;  // 当前group 所支持的 plugin有哪些
    };

    class GroupRoutingResolver {
       public:
        void configure(const std::vector<runtime::GroupRoutingConfig>& groupRouting);
        std::vector<GroupRoutingPlan> resolvePlans() const;

       private:
        std::vector<runtime::GroupRoutingConfig> groupRouting_;
    };
}  // namespace puppet::routing
