#pragma once

#include "puppet/orchestrator/orchestrator.hpp"
#include "puppet/runtime/runtime_config.hpp"

namespace puppet::routing {

    struct GroupRoutingPlan {
        orchestrator::GroupExecutionPlan gePlan_;
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