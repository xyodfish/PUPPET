#pragma once

#include "puppet/runtime/runtime_config.hpp"

#include <atomic>
#include <mutex>
namespace puppet::routing {

    struct GroupRoutingPlan {
        std::string bodyGroup;
        std::string ownerSourceId;
        std::string pluginId;
        std::string backendId;
        std::string mode;
        std::string controlSemantics;
        int32_t priority = 0;
        std::vector<runtime::PluginConfig> activedPlugins_;  // 当前group 所支持的 plugin有哪些
    };

    class GroupRoutingResolver {
       public:
        void updateGroupRouting(const std::vector<runtime::GroupRoutingConfig>& groupRouting);
        const std::vector<GroupRoutingPlan>& getPlans();
        const std::vector<GroupRoutingPlan>& getPlans() const;

       private:
        std::vector<GroupRoutingPlan> resolvePlans();
        void updatePlans();
        void updatePlans() const;
        bool plansChanged() const;
        void clearPlansChanged();
        void clearPlansChanged() const;

        std::vector<runtime::GroupRoutingConfig> groupRouting_;
        std::vector<GroupRoutingPlan> curPlans_;
        mutable std::atomic_bool plansChanged_ = true;
        mutable std::mutex planMtx_, routeMtx_;
    };
}  // namespace puppet::routing
