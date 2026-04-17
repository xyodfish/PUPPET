#include "puppet/runtime/teleop_runtime.hpp"

#include <iostream>

#include <glog/logging.h>

namespace puppet::runtime {

    bool TeleopRuntime::init(const std::string& configPath, std::string& error) {
        std::string configError;
        if (!RuntimeConfigLoader::loadFromYamlFile(configPath, config_, configError)) {
            error = "load runtime config failed: " + configError;
            return false;
        }

        return init(config_, error);
    }

    bool TeleopRuntime::init(const RuntimeConfig& runtimeConfig, std::string& error) {
        config_ = runtimeConfig;

        sourceManager_.configure(config_.sources);
        orchestrator_.configure(config_.groupRouting);

        std::string pipelineError;
        if (!pipeline_.configure(config_, &pipelineError)) {
            error = "configure retargeting pipeline failed: " + pipelineError;
            return false;
        }

        error.clear();
        return true;
    }

    bool TeleopRuntime::runOnce(std::string& error) {
        model::ControlIntent controlIntent;
        controlIntent.sequenceId = ++sequenceId_;
        bool hasAnyInputFrame    = false;

        const auto plans = orchestrator_.resolvePlans();
        for (const auto& plan : plans) {
            const auto frame = sourceManager_.getLatestFrame(plan.ownerSourceId);
            if (frame == nullptr) {
                continue;
            }
            hasAnyInputFrame = true;

            model::GroupControlIntent groupIntent;
            groupIntent.mode                   = plan.mode;
            groupIntent.backendHint            = plan.backendId;
            model::PrimitiveFrame runtimeFrame = *frame;
            if (pipeline_.requiresRobotState(plan.pipelineId, plan.controlSemantics)) {
                const bool hasFreshRobotState =
                    (robotStateSync_ != nullptr) && robotStateSync_->hasFreshState(config_.robotState.freshnessTimeoutMs);
                if (!hasFreshRobotState) {
                    static uint64_t noRobotStateWarnCount = 0;
                    ++noRobotStateWarnCount;
                    if ((noRobotStateWarnCount % 100ULL) == 1ULL) {
                        LOG(WARNING) << "TeleopRuntime skip plan due to stale robot state. pipeline=" << plan.pipelineId
                                     << " body_group=" << plan.bodyGroup
                                     << " robot_state_timeout_ms=" << config_.robotState.freshnessTimeoutMs;
                    }
                    continue;
                }
                const auto robotSnapshot = robotStateSync_->snapshot();
                runtimeFrame.jointStates.insert(runtimeFrame.jointStates.end(), robotSnapshot.frame.jointStates.begin(),
                                                robotSnapshot.frame.jointStates.end());
            }
            std::string pipelineError;
            if (!pipeline_.run(plan.pipelineId, runtimeFrame, plan.bodyGroup, plan.controlSemantics, &groupIntent, &pipelineError)) {
                error = "pipeline run failed: " + pipelineError;
                return false;
            }
            controlIntent.groupIntents.push_back(std::move(groupIntent));
        }

        lastControlIntent_     = controlIntent;
        const auto finalTarget = backend_.buildTarget(lastControlIntent_);
        // std::cout << "[teleop_runtime] seq=" << finalTarget.sequenceId << " groups=" << finalTarget.groups.size() << std::endl;
        if (!hasAnyInputFrame) {
            static uint64_t noInputCount = 0;
            ++noInputCount;
            if ((noInputCount % 200ULL) == 1ULL) {
                LOG(WARNING) << "TeleopRuntime has no input frame for resolved plans. plan_count=" << plans.size()
                             << " control_intent_groups=" << finalTarget.groups.size();
            }
        }

        error.clear();
        return true;
    }

}  // namespace puppet::runtime
