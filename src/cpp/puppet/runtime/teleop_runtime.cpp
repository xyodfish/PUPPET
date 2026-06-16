#include "puppet/runtime/teleop_runtime.hpp"

#include <iostream>

#include <glog/logging.h>

#include "puppet/common/logging.hpp"

namespace puppet::runtime {
    namespace {
        const runtime::PipelineConfig* selectActivePlugin(const routing::GroupRoutingPlan& routingPlan,
                                                          const model::PrimitiveFrame& frame) {
            const auto& activePlugins = routingPlan.activedPlugins_;
            if (activePlugins.empty()) {
                return nullptr;
            }

            if (!frame.context.pipelineId.empty()) {
                for (const auto& plugin : activePlugins) {
                    if (plugin.enabled && plugin.pipelineId == frame.context.pipelineId) {
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
        grSolver_.configure(config_.groupRouting);

        std::string pplErr;
        if (!pipeline_.configure(config_, pplErr)) {
            error = pplErr;
            return common::WrapError(error, "configure retargeting pipeline failed: ");
        }

        common::ClearError(error);
        PUPPET_LOG(INFO, "runtime_configured", "teleop_runtime", "init")
            << " sources=" << config_.sources.size() << " routes=" << config_.groupRouting.size()
            << " pipelines=" << config_.pipelines.size() << " backends=" << config_.backends.size();
        return true;
    }

    bool TeleopRuntime::runOnce(std::string& error) {
        model::ControlIntent controlIntent;
        controlIntent.sequenceId = ++sequenceId_;
        bool hasAnyInputFrame    = false;

        const auto plans = grSolver_.resolvePlans();
        for (const auto& routingPlan : plans) {
            auto plan        = routingPlan;
            const auto frame = sourceManager_.getLatestFrame(plan.ownerSourceId);
            if (frame == nullptr) {
                continue;
            }
            hasAnyInputFrame = true;

            if (const auto* activePlugin = selectActivePlugin(routingPlan, *frame); activePlugin != nullptr) {
                plan.pipelineId       = activePlugin->pipelineId;
                plan.mode             = activePlugin->pluginType;
                plan.controlSemantics = inferControlSemantics(activePlugin->pluginType, *frame, plan.controlSemantics);
            }

            model::GroupControlIntent groupIntent;
            groupIntent.mode                   = plan.mode;
            groupIntent.backendHint            = plan.backendId;
            model::PrimitiveFrame runtimeFrame = *frame;
            if (pipeline_.requiresRobotState(plan)) {
                const bool hasFreshRobotState =
                    (robotStateSync_ != nullptr) && robotStateSync_->hasFreshState(config_.robotState.freshnessTimeoutMs);
                if (!hasFreshRobotState) {
                    static uint64_t noRobotStateWarnCount = 0;
                    ++noRobotStateWarnCount;
                    if ((noRobotStateWarnCount % 100ULL) == 1ULL) {
                        PUPPET_LOG(WARNING, "plan_skipped_stale_robot_state", "teleop_runtime", "run_once")
                            << " pipeline_id=" << plan.pipelineId << " body_group=" << plan.bodyGroup << " source_id=" << plan.ownerSourceId
                            << " timeout_ms=" << config_.robotState.freshnessTimeoutMs;
                    }
                    continue;
                }
                const auto robotSnapshot = robotStateSync_->snapshot();
                runtimeFrame.jointStates.insert(runtimeFrame.jointStates.end(), robotSnapshot.frame.jointStates.begin(),
                                                robotSnapshot.frame.jointStates.end());
            }
            std::string pplErr;
            PUPPET_VLOG(2, "pipeline_dispatch", "teleop_runtime", "run_once")
                << " pipeline_id=" << plan.pipelineId << " body_group=" << plan.bodyGroup << " source_id=" << plan.ownerSourceId
                << " seq=" << controlIntent.sequenceId;
            if (!pipeline_.run(plan, runtimeFrame, &groupIntent, pplErr)) {
                error = pplErr;
                PUPPET_LOG(ERROR, "pipeline_run_failed", "teleop_runtime", "run_once")
                    << " pipeline_id=" << plan.pipelineId << " body_group=" << plan.bodyGroup << " source_id=" << plan.ownerSourceId
                    << " seq=" << controlIntent.sequenceId << " error=" << error;
                return common::WrapError(error, "pipeline run failed: ");
            }
            controlIntent.groupIntents.push_back(std::move(groupIntent));
        }

        lastControlIntent_     = controlIntent;
        const auto finalTarget = backend_.buildTarget(lastControlIntent_);
        if (!hasAnyInputFrame) {
            static uint64_t noInputCount = 0;
            ++noInputCount;
            if ((noInputCount % 200ULL) == 1ULL) {
                PUPPET_LOG(WARNING, "source_frame_missing", "teleop_runtime", "run_once")
                    << " plan_count=" << plans.size() << " control_intent_groups=" << finalTarget.groups.size();
            }
        }

        common::ClearError(error);
        return true;
    }

}  // namespace puppet::runtime
