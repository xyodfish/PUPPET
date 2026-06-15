#include "puppet/retargeting/core/retargeting_pipeline.hpp"

#include <glog/logging.h>

#include "puppet/common/logging.hpp"
#include "puppet/retargeting/native/gmr_retargeting_plugin.hpp"
#include "puppet/retargeting/native/single_chain_ik_retargeting_plugin.hpp"

namespace puppet::retargeting {
    namespace {
        bool supportsGroup(const std::string& pluginType, const std::string& bodyGroup) {
            if (pluginType == "direct_pass") {
                return true;
            }
            if (pluginType == "gmr") {
                return bodyGroup == "whole_body";
            }
            if (pluginType == "single_chain_ik" || pluginType == "single_chain_ik_velocity") {
                return bodyGroup != "whole_body";
            }
            return false;
        }

        bool supportsSemantics(const std::string& pluginType, const std::string& controlSemantics) {
            if (pluginType == "direct_pass") {
                return controlSemantics == "joint_absolute" || controlSemantics == "joint_delta";
            }
            if (pluginType == "gmr") {
                return controlSemantics == "cartesian_absolute" || controlSemantics == "cartesian_delta";
            }
            if (pluginType == "single_chain_ik" || pluginType == "single_chain_ik_velocity") {
                return controlSemantics == "cartesian_absolute" || controlSemantics == "cartesian_delta" ||
                       controlSemantics == "cartesian_velocity";
            }
            return false;
        }

        bool isPluginEnabled(const runtime::RuntimeConfig& config, const std::string& pluginType) {
            if (pluginType == "direct_pass") {
                return true;
            }
            if (pluginType == "gmr") {
                return config.gmr.enabled;
            }
            if (pluginType == "single_chain_ik") {
                return config.singleChainIk.enabled;
            }
            if (pluginType == "single_chain_ik_velocity") {
                return config.singleChainIk.enabled;
            }
            return false;
        }
    }  // namespace

    bool RetargetingPipeline::configure(const runtime::RuntimeConfig& config, std::string& error) {
        plugins_.clear();
        pipelineTypes_.clear();

        for (const auto& route : config.groupRouting) {
            const auto pipelineIt = config.pipelineMap.find(route.pipelineId);
            if (pipelineIt == config.pipelineMap.end()) {
                return common::Fail(error, "group_routing references missing pipeline: " + route.pipelineId);
            }
            if (!pipelineIt->second.enabled) {
                return common::Fail(error, "group_routing references disabled pipeline: " + route.pipelineId);
            }
        }

        for (const auto& pipelineConfig : config.pipelines) {
            if (!pipelineConfig.enabled) {
                PUPPET_LOG(INFO, "pipeline_skipped_disabled", "retargeting_pipeline", "configure")
                    << " pipeline_id=" << pipelineConfig.pipelineId << " plugin_type=" << pipelineConfig.pluginType;
                continue;
            }

            if (!isPluginEnabled(config, pipelineConfig.pluginType)) {
                return common::Fail(
                    error, "pipeline " + pipelineConfig.pipelineId + " references disabled plugin type: " + pipelineConfig.pluginType);
            }

            RetargetingPluginPtr plugin;
            if (pipelineConfig.pluginType == "direct_pass") {
                plugin = std::make_shared<DirectPassThroughPlugin>();
            } else if (pipelineConfig.pluginType == "gmr") {
                plugin = std::make_shared<GmrRetargetingPlugin>();
            } else if (pipelineConfig.pluginType == "single_chain_ik_velocity") {
                PUPPET_LOG(WARNING, "deprecated_plugin_type", "retargeting_pipeline", "configure")
                    << " pipeline_id=" << pipelineConfig.pipelineId << " plugin_type=single_chain_ik_velocity"
                    << " fallback=single_chain_ik";
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else if (pipelineConfig.pluginType == "single_chain_ik") {
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else {
                return common::Fail(error, "unknown plugin type: " + pipelineConfig.pluginType);
            }

            std::string pluginError;
            if (!plugin->configure(config, pluginError)) {
                error = pluginError;
                return common::WrapError(error, "configure plugin " + pipelineConfig.pluginType + " failed: ");
            }
            plugins_[pipelineConfig.pipelineId]       = std::move(plugin);
            pipelineTypes_[pipelineConfig.pipelineId] = pipelineConfig.pluginType;
            PUPPET_LOG(INFO, "pipeline_configured", "retargeting_pipeline", "configure")
                << " pipeline_id=" << pipelineConfig.pipelineId << " plugin_type=" << pipelineConfig.pluginType;
        }

        common::ClearError(error);
        return true;
    }

    bool RetargetingPipeline::requiresRobotState(const orchestrator::GroupExecutionPlan& plan) const {
        const auto it = pipelineTypes_.find(plan.pipelineId);
        if (it == pipelineTypes_.end()) {
            return false;
        }
        const std::string& pluginType = it->second;
        if (pluginType == "single_chain_ik" && plan.controlSemantics == "cartesian_delta") {
            return true;
        }
        if (pluginType == "single_chain_ik" && plan.controlSemantics == "cartesian_velocity") {
            return true;
        }
        const auto pluginIt = plugins_.find(plan.pipelineId);
        if (pluginIt == plugins_.end()) {
            return false;
        }
        return pluginIt->second->requiresRobotState();
    }

    bool RetargetingPipeline::run(const orchestrator::GroupExecutionPlan& plan, const model::PrimitiveFrame& frame,
                                  model::GroupControlIntent* output, std::string& error) const {
        const auto pluginIt = plugins_.find(plan.pipelineId);
        if (pluginIt == plugins_.end()) {
            return common::Fail(error, "pipeline not found: " + plan.pipelineId);
        }

        const auto typeIt = pipelineTypes_.find(plan.pipelineId);
        if (typeIt == pipelineTypes_.end()) {
            return common::Fail(error, "pipeline type not found: " + plan.pipelineId);
        }

        const std::string& pluginType = typeIt->second;
        PUPPET_VLOG(2, "plugin_dispatch", "retargeting_pipeline", "run")
            << " pipeline_id=" << plan.pipelineId << " plugin_type=" << pluginType << " body_group=" << plan.bodyGroup
            << " control_semantics=" << plan.controlSemantics;

        if (!supportsGroup(pluginType, plan.bodyGroup)) {
            return common::Fail(
                error, "pipeline " + plan.pipelineId + " plugin " + pluginType + " does not support body_group: " + plan.bodyGroup);
        }

        if (!supportsSemantics(pluginType, plan.controlSemantics)) {
            return common::Fail(error, "pipeline " + plan.pipelineId + " plugin " + pluginType +
                                           " does not support control_semantics: " + plan.controlSemantics);
        }

        return pluginIt->second->process(frame, plan.bodyGroup, output, error);
    }

}  // namespace puppet::retargeting
