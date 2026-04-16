#include "puppet/orchestrator/orchestrator.hpp"

namespace puppet::orchestrator {

    void Orchestrator::configure(const std::vector<runtime::GroupRoutingConfig>& groupRouting) {
        groupRouting_ = groupRouting;
    }

    std::vector<GroupExecutionPlan> Orchestrator::resolvePlans() const {
        std::vector<GroupExecutionPlan> plans;
        plans.reserve(groupRouting_.size());
        for (const auto& route : groupRouting_) {
            GroupExecutionPlan plan;
            plan.bodyGroup     = route.bodyGroup;
            plan.ownerSourceId = route.ownerSourceId;
            plan.pipelineId    = route.pipelineId;
            plan.backendId     = route.backendId;
            plan.mode          = route.mode;
            plans.push_back(std::move(plan));
        }
        return plans;
    }

}  // namespace puppet::orchestrator
