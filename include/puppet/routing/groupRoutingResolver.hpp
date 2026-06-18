#pragma once

#include "puppet/runtime/runtime_config.hpp"

#include <atomic>
#include <mutex>
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
        void updateGroupRouting(const std::vector<runtime::GroupRoutingConfig>& groupRouting);
        const std::vector<GroupRoutingPlan>& getPlans();

       private:
        std::vector<GroupRoutingPlan> resolvePlans();
        void updatePlans();
        bool plansChanged() const;
        void clearPlansChanged();

        std::vector<runtime::GroupRoutingConfig> groupRouting_;
        std::vector<GroupRoutingPlan> curPlans_;
        std::atomic_bool plansChanged_ = true;
        std::mutex planMtx_, routeMtx_;
    };
}  // namespace puppet::routing
