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

            candidate.activedPlugins_ = route.activedPlugins;
            auto it                   = routingPlans.find(route.bodyGroup);
            if (it == routingPlans.end() || candidate.priority > it->second.priority) {
                routingPlans[route.bodyGroup] = std::move(candidate);
            }
        }

        std::vector<GroupRoutingPlan> plans;
        plans.reserve(routingPlans.size());
        for (auto& kv : routingPlans) {
            const auto& plan = kv.second;
            PUPPET_VLOG(1, "plan_resolved", "group_routing_resolver", "resolve_plans")
                << " body_group=" << plan.bodyGroup << " source_id=" << plan.ownerSourceId << " pipeline_id=" << plan.pipelineId
                << " backend=" << plan.backendId << " priority=" << plan.priority << " mode=" << plan.mode
                << " control_semantics=" << plan.controlSemantics;
            plans.push_back(std::move(kv.second));
        }

        return plans;
    }
}  // namespace puppet::routing
