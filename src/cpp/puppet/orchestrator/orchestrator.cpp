#include "puppet/orchestrator/orchestrator.hpp"

#include <unordered_map>

#include <glog/logging.h>

#include "puppet/common/logging.hpp"

namespace puppet::orchestrator {

    void Orchestrator::configure(const std::vector<runtime::GroupRoutingConfig>& groupRouting) {
        groupRouting_ = groupRouting;
    }

    std::vector<GroupExecutionPlan> Orchestrator::resolvePlans() const {
        constexpr int32_t kWholeBodyBonus = 10000;
        std::unordered_map<std::string, GroupExecutionPlan> selectedPlans;

        for (const auto& route : groupRouting_) {
            if (!route.enabled) {
                continue;
            }
            GroupExecutionPlan candidate;
            candidate.bodyGroup        = route.bodyGroup;
            candidate.ownerSourceId    = route.ownerSourceId;
            candidate.pipelineId       = route.pipelineId;
            candidate.backendId        = route.backendId;
            candidate.mode             = route.mode;
            candidate.controlSemantics = route.controlSemantics;
            candidate.priority         = route.priority;
            if (route.bodyGroup == "whole_body") {
                candidate.priority += kWholeBodyBonus;
            }

            auto it = selectedPlans.find(route.bodyGroup);
            if (it == selectedPlans.end() || candidate.priority > it->second.priority) {
                selectedPlans[route.bodyGroup] = std::move(candidate);
            }
        }

        std::vector<GroupExecutionPlan> plans;
        plans.reserve(selectedPlans.size());
        for (auto& kv : selectedPlans) {
            PUPPET_VLOG(1, "plan_resolved", "orchestrator", "resolve_plans")
                << " body_group=" << kv.second.bodyGroup << " source_id=" << kv.second.ownerSourceId
                << " pipeline_id=" << kv.second.pipelineId << " backend=" << kv.second.backendId << " priority=" << kv.second.priority
                << " mode=" << kv.second.mode << " control_semantics=" << kv.second.controlSemantics;
            plans.push_back(std::move(kv.second));
        }
        return plans;
    }

}  // namespace puppet::orchestrator
