#include "puppet/routing/groupRoutingResolver.hpp"

#include <unordered_map>

#include "puppet/common/logging.hpp"

namespace puppet::routing {
    void GroupRoutingResolver::configure(const std::vector<runtime::GroupRoutingConfig>& groupRouting) {
        groupRouting_ = groupRouting;
    }

    std::vector<GroupRoutingPlan> GroupRoutingResolver::resolvePlans() const {
        constexpr int32_t kWholeBodyBonus = 10000;
        std::unordered_map<std::string, GroupRoutingPlan> routingPlans;

        for (const auto& route : groupRouting_) {
            if (!route.enabled) {
                continue;
            }

            GroupRoutingPlan candidate;
            auto& gePlan            = candidate.gePlan_;
            gePlan.bodyGroup        = route.bodyGroup;
            gePlan.ownerSourceId    = route.ownerSourceId;
            gePlan.pipelineId       = route.pipelineId;
            gePlan.backendId        = route.backendId;
            gePlan.mode             = route.mode;
            gePlan.controlSemantics = route.controlSemantics;
            gePlan.priority         = route.priority;
            if (route.bodyGroup == "whole_body") {
                gePlan.priority += kWholeBodyBonus;
            }

            candidate.activedPlugins_ = route.activedPlugins;
            auto it                   = routingPlans.find(route.bodyGroup);
            if (it == routingPlans.end() || gePlan.priority > it->second.gePlan_.priority) {
                routingPlans[route.bodyGroup] = std::move(candidate);
            }
        }

        std::vector<GroupRoutingPlan> plans;
        plans.reserve(routingPlans.size());
        for (auto& kv : routingPlans) {
            const auto& gePlan = kv.second.gePlan_;
            PUPPET_VLOG(1, "plan_resolved", "orchestrator", "resolve_plans")
                << " body_group=" << gePlan.bodyGroup << " source_id=" << gePlan.ownerSourceId << " pipeline_id=" << gePlan.pipelineId
                << " backend=" << gePlan.backendId << " priority=" << gePlan.priority << " mode=" << gePlan.mode
                << " control_semantics=" << gePlan.controlSemantics;
            plans.push_back(std::move(kv.second));
        }

        return plans;
    }
}  // namespace puppet::routing