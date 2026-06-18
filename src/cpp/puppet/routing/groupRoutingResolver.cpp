#include "puppet/routing/groupRoutingResolver.hpp"

#include <unordered_map>

#include "puppet/common/logging.hpp"

#include <mutex>
namespace puppet::routing {
    void GroupRoutingResolver::updateGroupRouting(const std::vector<runtime::GroupRoutingConfig>& groupRouting) {
        std::unique_lock<std::mutex> lock(routeMtx_);
        groupRouting_ = groupRouting;
        plansChanged_.store(true, std::memory_order_release);
    }

    void GroupRoutingResolver::updatePlans() {
        std::unique_lock<std::mutex> lock(planMtx_);
        if (!plansChanged()) {
            return;
        }
        curPlans_ = resolvePlans();
        clearPlansChanged();
    }

    std::vector<GroupRoutingPlan> GroupRoutingResolver::resolvePlans() {
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

    bool GroupRoutingResolver::plansChanged() const {
        return plansChanged_.load(std::memory_order_acquire);
    }

    void GroupRoutingResolver::clearPlansChanged() {
        plansChanged_.store(false, std::memory_order_release);
    }

    const std::vector<GroupRoutingPlan>& GroupRoutingResolver::getPlans() {
        updatePlans();
        return curPlans_;
    }
}  // namespace puppet::routing
