#include "puppet/retargeting/core/retargeting_pipeline.hpp"

#include <glog/logging.h>

#include "puppet/retargeting/native/gmr_retargeting_plugin.hpp"
#include "puppet/retargeting/native/single_chain_ik_retargeting_plugin.hpp"

namespace puppet::retargeting {
    namespace {
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
            return false;
        }
    }  // namespace

    bool RetargetingPipeline::configure(const runtime::RuntimeConfig& config, std::string* error) {
        plugins_.clear();

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
            plugins_[pipelineConfig.pipelineId] = std::move(plugin);
            LOG(INFO) << "RetargetingPipeline configured pipeline_id=" << pipelineConfig.pipelineId
                      << " plugin_type=" << pipelineConfig.pluginType;
        }

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    bool RetargetingPipeline::run(const std::string& pipelineId, const model::PrimitiveFrame& frame, const std::string& bodyGroup,
                                  model::GroupControlIntent* output, std::string* error) const {
        const auto pluginIt = plugins_.find(pipelineId);
        if (pluginIt == plugins_.end()) {
            if (error != nullptr) {
                *error = "pipeline not found: " + pipelineId;
            }
            return false;
        }
        return pluginIt->second->process(frame, bodyGroup, output, error);
    }

}  // namespace puppet::retargeting
