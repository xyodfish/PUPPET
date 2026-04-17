#include "puppet/retargeting/core/retargeting_pipeline.hpp"

#include <glog/logging.h>

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

    bool RetargetingPipeline::configure(const runtime::RuntimeConfig& config, std::string* error) {
        plugins_.clear();
        pipelineTypes_.clear();

        for (const auto& route : config.groupRouting) {
            const auto pipelineIt = config.pipelineMap.find(route.pipelineId);
            if (pipelineIt == config.pipelineMap.end()) {
                if (error != nullptr) {
                    *error = "group_routing references missing pipeline: " + route.pipelineId;
                }
                return false;
            }
            if (!pipelineIt->second.enabled) {
                if (error != nullptr) {
                    *error = "group_routing references disabled pipeline: " + route.pipelineId;
                }
                return false;
            }
        }

        for (const auto& pipelineConfig : config.pipelines) {
            if (!pipelineConfig.enabled) {
                LOG(INFO) << "RetargetingPipeline skip disabled pipeline_id=" << pipelineConfig.pipelineId
                          << " plugin_type=" << pipelineConfig.pluginType;
                continue;
            }

            if (!isPluginEnabled(config, pipelineConfig.pluginType)) {
                if (error != nullptr) {
                    *error = "pipeline " + pipelineConfig.pipelineId + " references disabled plugin type: " + pipelineConfig.pluginType;
                }
                return false;
            }

            RetargetingPluginPtr plugin;
            if (pipelineConfig.pluginType == "direct_pass") {
                plugin = std::make_shared<DirectPassThroughPlugin>();
            } else if (pipelineConfig.pluginType == "gmr") {
                plugin = std::make_shared<GmrRetargetingPlugin>();
            } else if (pipelineConfig.pluginType == "single_chain_ik_velocity") {
                LOG(WARNING) << "pipeline_id=" << pipelineConfig.pipelineId
                             << " uses deprecated plugin type single_chain_ik_velocity, fallback to single_chain_ik";
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else if (pipelineConfig.pluginType == "single_chain_ik") {
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else {
                if (error != nullptr) {
                    *error = "unknown plugin type: " + pipelineConfig.pluginType;
                }
                return false;
            }

            std::string pluginError;
            if (!plugin->configure(config, &pluginError)) {
                if (error != nullptr) {
                    *error = "configure plugin " + pipelineConfig.pluginType + " failed: " + pluginError;
                }
                return false;
            }
            plugins_[pipelineConfig.pipelineId]       = std::move(plugin);
            pipelineTypes_[pipelineConfig.pipelineId] = pipelineConfig.pluginType;
            LOG(INFO) << "RetargetingPipeline configured pipeline_id=" << pipelineConfig.pipelineId
                      << " plugin_type=" << pipelineConfig.pluginType;
        }

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    bool RetargetingPipeline::requiresRobotState(const std::string& pipelineId, const std::string& controlSemantics) const {
        const auto it = pipelineTypes_.find(pipelineId);
        if (it == pipelineTypes_.end()) {
            return false;
        }
        const std::string& pluginType = it->second;
        if (pluginType == "single_chain_ik" && controlSemantics == "cartesian_delta") {
            return true;
        }
        if (pluginType == "single_chain_ik" && controlSemantics == "cartesian_velocity") {
            return true;
        }
        if (pluginType == "single_chain_ik_velocity") {
            return controlSemantics == "cartesian_delta" || controlSemantics == "cartesian_velocity";
        }
        const auto pluginIt = plugins_.find(pipelineId);
        if (pluginIt == plugins_.end()) {
            return false;
        }
        return pluginIt->second->requiresRobotState();
    }

    bool RetargetingPipeline::run(const std::string& pipelineId, const model::PrimitiveFrame& frame, const std::string& bodyGroup,
                                  const std::string& controlSemantics, model::GroupControlIntent* output, std::string* error) const {
        const auto pluginIt = plugins_.find(pipelineId);
        if (pluginIt == plugins_.end()) {
            if (error != nullptr) {
                *error = "pipeline not found: " + pipelineId;
            }
            return false;
        }
        const auto typeIt = pipelineTypes_.find(pipelineId);
        if (typeIt == pipelineTypes_.end()) {
            if (error != nullptr) {
                *error = "pipeline type not found: " + pipelineId;
            }
            return false;
        }
        const std::string& pluginType = typeIt->second;
        if (!supportsGroup(pluginType, bodyGroup)) {
            if (error != nullptr) {
                *error = "pipeline " + pipelineId + " plugin " + pluginType + " does not support body_group: " + bodyGroup;
            }
            return false;
        }
        if (!supportsSemantics(pluginType, controlSemantics)) {
            if (error != nullptr) {
                *error = "pipeline " + pipelineId + " plugin " + pluginType + " does not support control_semantics: " + controlSemantics;
            }
            return false;
        }
        return pluginIt->second->process(frame, bodyGroup, output, error);
    }

}  // namespace puppet::retargeting
