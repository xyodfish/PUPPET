#include "puppet/retargeting/core/retargeting_pipeline.hpp"

#include "puppet/retargeting/native/gmr_retargeting_plugin.hpp"

namespace puppet::retargeting {

    bool RetargetingPipeline::configure(const runtime::RuntimeConfig& config, std::string* error) {
        plugins_.clear();

        for (const auto& pipelineConfig : config.pipelines) {
            RetargetingPluginPtr plugin;
            if (pipelineConfig.pluginType == "direct_pass") {
                plugin = std::make_shared<DirectPassThroughPlugin>();
            } else if (pipelineConfig.pluginType == "gmr") {
                plugin = std::make_shared<GmrRetargetingPlugin>();
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
