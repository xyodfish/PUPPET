#include "puppet/runtime/teleop_runtime.hpp"

#include <iostream>

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

        const auto plans = orchestrator_.resolvePlans();
        for (const auto& plan : plans) {
            const auto frame = sourceManager_.getLatestFrame(plan.ownerSourceId);
            if (frame == nullptr) {
                continue;
            }

            model::GroupControlIntent groupIntent;
            groupIntent.mode        = plan.mode;
            groupIntent.backendHint = plan.backendId;
            std::string pipelineError;
            if (!pipeline_.run(plan.pipelineId, *frame, plan.bodyGroup, &groupIntent, &pipelineError)) {
                error = "pipeline run failed: " + pipelineError;
                return false;
            }
            controlIntent.groupIntents.push_back(std::move(groupIntent));
        }

        lastControlIntent_     = controlIntent;
        const auto finalTarget = backend_.buildTarget(lastControlIntent_);
        // std::cout << "[teleop_runtime] seq=" << finalTarget.sequenceId << " groups=" << finalTarget.groups.size() << std::endl;

        error.clear();
        return true;
    }

}  // namespace puppet::runtime
