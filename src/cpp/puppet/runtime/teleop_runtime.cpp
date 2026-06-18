#include "puppet/runtime/teleop_runtime.hpp"

#include <iostream>

#include <glog/logging.h>

#include "puppet/common/logging.hpp"

namespace puppet::runtime {
    namespace {
        template <typename Primitive>
        bool matchesBodyGroup(const Primitive& primitive, const std::string& bodyGroup) {
            return primitive.meta.bodyGroup.empty() || primitive.meta.bodyGroup == bodyGroup;
        }

        template <typename PrimitiveVec>
        PrimitiveVec filterPrimitiveVec(const PrimitiveVec& primitives, const std::string& bodyGroup) {
            PrimitiveVec filtered;
            filtered.reserve(primitives.size());
            for (const auto& primitive : primitives) {
                if (matchesBodyGroup(primitive, bodyGroup)) {
                    filtered.push_back(primitive);
                }
            }
            return filtered;
        }

        // 对于plan的 bodyGroup 把和输入的PrimitiveFrame中所描述的 用于 当前bodyGroup的 所有pose twist之类的数据都提取出来。
        // 例如 当前plan的 bodyGroup 为 left_arm 则把 frame.poses 中关于 left_arm 的数据提取出来。拼成一个用于retargeting的 PrimitiveFrame
        model::PrimitiveFrame buildRuntimeFrameForPlan(const model::PrimitiveFrame& frame, const std::string& bodyGroup) {
            if (bodyGroup.empty() || bodyGroup == "whole_body") {
                return frame;
            }

            model::PrimitiveFrame filtered = frame;
            filtered.poses                 = filterPrimitiveVec(frame.poses, bodyGroup);
            filtered.twists                = filterPrimitiveVec(frame.twists, bodyGroup);
            filtered.jointStates           = filterPrimitiveVec(frame.jointStates, bodyGroup);
            filtered.jointCommands         = filterPrimitiveVec(frame.jointCommands, bodyGroup);
            filtered.scalars               = filterPrimitiveVec(frame.scalars, bodyGroup);
            filtered.booleans              = filterPrimitiveVec(frame.booleans, bodyGroup);
            filtered.modes                 = filterPrimitiveVec(frame.modes, bodyGroup);
            filtered.planarMotions         = filterPrimitiveVec(frame.planarMotions, bodyGroup);
            filtered.skeletons             = filterPrimitiveVec(frame.skeletons, bodyGroup);
            filtered.landmarkSets          = filterPrimitiveVec(frame.landmarkSets, bodyGroup);
            return filtered;
        }

        std::string selectRequestedPluginId(const routing::GroupRoutingPlan& routingPlan, const model::PrimitiveFrame& frame) {
            const auto it = frame.context.groupPluginIds.find(routingPlan.bodyGroup);
            if (it != frame.context.groupPluginIds.end() && !it->second.empty()) {
                return it->second;
            }
            return frame.context.pluginId;
        }

        const runtime::PluginConfig* selectActivePlugin(const routing::GroupRoutingPlan& routingPlan, const model::PrimitiveFrame& frame) {
            const auto& activePlugins = routingPlan.activedPlugins_;
            if (activePlugins.empty()) {
                return nullptr;
            }

            const std::string requestedPluginId = selectRequestedPluginId(routingPlan, frame);
            if (!requestedPluginId.empty()) {
                for (const auto& plugin : activePlugins) {
                    if (plugin.enabled && plugin.pluginId == requestedPluginId) {
                        return &plugin;
                    }
                }
            }

            for (const auto& plugin : activePlugins) {
                if (plugin.enabled) {
                    return &plugin;
                }
            }
            return nullptr;
        }

        std::string inferControlSemantics(const std::string& pluginType, const model::PrimitiveFrame& frame, const std::string& fallback) {
            if (pluginType == "direct_pass" && !frame.jointCommands.empty()) {
                return "joint_absolute";
            }
            if ((pluginType == "single_chain_ik" || pluginType == "single_chain_ik_velocity") && !frame.poses.empty()) {
                return "cartesian_absolute";
            }
            return fallback;
        }
    }  // namespace

    bool TeleopRuntime::init(const std::string& configPath, std::string& error) {
        std::string configError;
        if (!RuntimeConfigLoader::loadFromYamlFile(configPath, config_, configError)) {
            error = configError;
            return common::WrapError(error, "load runtime config failed: ");
        }

        return init(config_, error);
    }

    bool TeleopRuntime::init(const RuntimeConfig& runtimeConfig, std::string& error) {
        config_ = runtimeConfig;

        sourceManager_.configure(config_.sources);
        groupRouteSolver_.updateGroupRouting(config_.groupRouting);

        std::string pluginErr;
        if (!pipeline_.configure(config_, pluginErr)) {
            error = pluginErr;
            return common::WrapError(error, "configure retargeting pipeline failed: ");
        }

        common::ClearError(error);
        PUPPET_LOG(INFO, "runtime_configured", "teleop_runtime", "init")
            << " sources=" << config_.sources.size() << " routes=" << config_.groupRouting.size() << " plugins=" << config_.plugins.size()
            << " backends=" << config_.backends.size();
        return true;
    }

    bool TeleopRuntime::preparePlanFrame(const routing::GroupRoutingPlan& routingPlan, PlanInput* planInput) const {
        if (planInput == nullptr) {
            return false;
        }

        const auto frame = sourceManager_.getLatestFrame(routingPlan.ownerSourceId);
        if (frame == nullptr) {
            return false;
        }

        planInput->resolvedPlan = routingPlan;
        planInput->runtimeFrame = buildRuntimeFrameForPlan(*frame, routingPlan.bodyGroup);
        if (const auto* activePlugin = selectActivePlugin(planInput->resolvedPlan, *frame); activePlugin != nullptr) {
            planInput->resolvedPlan.pluginId = activePlugin->pluginId;
            planInput->resolvedPlan.mode     = activePlugin->pluginType;
            planInput->resolvedPlan.controlSemantics =
                inferControlSemantics(activePlugin->pluginType, planInput->runtimeFrame, planInput->resolvedPlan.controlSemantics);
        }
        return true;
    }

    bool TeleopRuntime::appendRobotStateIfNeeded(const routing::GroupRoutingPlan& plan, model::PrimitiveFrame* runtimeFrame) const {
        if (runtimeFrame == nullptr || !pipeline_.requiresRobotState(plan)) {
            return true;
        }

        const bool hasFreshRobotState =
            (robotStateSync_ != nullptr) && robotStateSync_->hasFreshState(config_.robotState.freshnessTimeoutMs);
        if (!hasFreshRobotState) {
            static uint64_t noRobotStateWarnCount = 0;
            ++noRobotStateWarnCount;
            if ((noRobotStateWarnCount % 100ULL) == 1ULL) {
                PUPPET_LOG(WARNING, "plan_skipped_stale_robot_state", "teleop_runtime", "run_once")
                    << " plugin_id=" << plan.pluginId << " body_group=" << plan.bodyGroup << " source_id=" << plan.ownerSourceId
                    << " timeout_ms=" << config_.robotState.freshnessTimeoutMs;
            }
            return false;
        }

        const auto robotSnapshot = robotStateSync_->snapshot();
        runtimeFrame->jointStates.insert(runtimeFrame->jointStates.end(), robotSnapshot.frame.jointStates.begin(),
                                         robotSnapshot.frame.jointStates.end());
        return true;
    }

    std::vector<TeleopRuntime::PlanInput> TeleopRuntime::collectPlanInputs(bool* hasAnyInputFrame) const {
        std::vector<PlanInput> planInputs;
        if (hasAnyInputFrame == nullptr) {
            return planInputs;
        }

        *hasAnyInputFrame = false;
        const auto& plans = groupRouteSolver_.getPlans();
        planInputs.reserve(plans.size());
        for (const auto& routingPlan : plans) {
            PlanInput planInput;
            if (!preparePlanFrame(routingPlan, &planInput)) {
                continue;
            }
            *hasAnyInputFrame = true;
            planInputs.push_back(std::move(planInput));
        }
        return planInputs;
    }

    bool TeleopRuntime::executePlans(const std::vector<PlanInput>& planInputs, model::ControlIntent* controlIntent, std::string& error) {
        if (controlIntent == nullptr) {
            error = "executePlans output pointer is null";
            return false;
        }

        for (const auto& planInput : planInputs) {
            model::PrimitiveFrame runtimeFrame = planInput.runtimeFrame;
            if (!appendRobotStateIfNeeded(planInput.resolvedPlan, &runtimeFrame)) {
                continue;
            }

            model::GroupControlIntent groupIntent;
            groupIntent.mode        = planInput.resolvedPlan.mode;
            groupIntent.backendHint = planInput.resolvedPlan.backendId;

            std::string pluginErr;
            PUPPET_VLOG(2, "plugin_dispatch", "teleop_runtime", "run_once")
                << " plugin_id=" << planInput.resolvedPlan.pluginId << " body_group=" << planInput.resolvedPlan.bodyGroup
                << " source_id=" << planInput.resolvedPlan.ownerSourceId << " seq=" << controlIntent->sequenceId;
            if (!pipeline_.run(planInput.resolvedPlan, runtimeFrame, &groupIntent, pluginErr)) {
                error = pluginErr;
                PUPPET_LOG(ERROR, "plugin_run_failed", "teleop_runtime", "run_once")
                    << " plugin_id=" << planInput.resolvedPlan.pluginId << " body_group=" << planInput.resolvedPlan.bodyGroup
                    << " source_id=" << planInput.resolvedPlan.ownerSourceId << " seq=" << controlIntent->sequenceId << " error=" << error;
                return common::WrapError(error, "plugin run failed: ");
            }

            controlIntent->groupIntents.push_back(std::move(groupIntent));
        }
        return true;
    }

    void TeleopRuntime::finalizeControlIntent(model::ControlIntent controlIntent, bool hasAnyInputFrame) {
        lastControlIntent_     = std::move(controlIntent);
        const auto finalTarget = backend_.buildTarget(lastControlIntent_);
        if (hasAnyInputFrame) {
            return;
        }

        static uint64_t noInputCount = 0;
        ++noInputCount;
        if ((noInputCount % 200ULL) == 1ULL) {
            PUPPET_LOG(WARNING, "source_frame_missing", "teleop_runtime", "run_once")
                << " plan_count=" << groupRouteSolver_.getPlans().size() << " control_intent_groups=" << finalTarget.groups.size();
        }
    }

    bool TeleopRuntime::runOnce(std::string& error) {
        model::ControlIntent controlIntent;
        controlIntent.sequenceId = ++sequenceId_;
        bool hasAnyInputFrame    = false;

        const auto planInputs = collectPlanInputs(&hasAnyInputFrame);
        if (!executePlans(planInputs, &controlIntent, error)) {
            return false;
        }
        finalizeControlIntent(std::move(controlIntent), hasAnyInputFrame);

        common::ClearError(error);
        return true;
    }

}  // namespace puppet::runtime
