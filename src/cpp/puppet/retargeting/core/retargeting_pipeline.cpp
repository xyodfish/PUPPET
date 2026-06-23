#include "puppet/retargeting/core/retargeting_pipeline.hpp"

#include <glog/logging.h>

#include "puppet/common/logging.hpp"
#include "puppet/retargeting/native/gmr/gmr_retargeting_plugin.hpp"
#include "puppet/retargeting/native/single_chain_ik/single_chain_ik_retargeting_plugin.hpp"

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
        pluginTypes_.clear();

        for (const auto& route : config.groupRouting) {
            const auto pluginIt = config.pluginMap.find(route.pluginId);
            if (pluginIt == config.pluginMap.end()) {
                return common::Fail(error, "group_routing references missing plugin: " + route.pluginId);
            }
            if (!pluginIt->second.enabled) {
                return common::Fail(error, "group_routing references disabled plugin: " + route.pluginId);
            }
        }

        for (const auto& pluginConfig : config.plugins) {
            if (!pluginConfig.enabled) {
                PUPPET_LOG(INFO, "plugin_skipped_disabled", "retargeting_pipeline", "configure")
                    << " plugin_id=" << pluginConfig.pluginId << " plugin_type=" << pluginConfig.pluginType;
                continue;
            }

            if (!isPluginEnabled(config, pluginConfig.pluginType)) {
                return common::Fail(error,
                                    "plugin " + pluginConfig.pluginId + " references disabled plugin type: " + pluginConfig.pluginType);
            }

            RetargetingPluginPtr plugin;
            if (pluginConfig.pluginType == "direct_pass") {
                plugin = std::make_shared<DirectPassThroughPlugin>();
            } else if (pluginConfig.pluginType == "gmr") {
                plugin = std::make_shared<GmrRetargetingPlugin>();
            } else if (pluginConfig.pluginType == "single_chain_ik_velocity") {
                PUPPET_LOG(WARNING, "deprecated_plugin_type", "retargeting_pipeline", "configure")
                    << " plugin_id=" << pluginConfig.pluginId << " plugin_type=single_chain_ik_velocity"
                    << " fallback=single_chain_ik";
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else if (pluginConfig.pluginType == "single_chain_ik") {
                plugin = std::make_shared<SingleChainIkRetargetingPlugin>();
            } else {
                return common::Fail(error, "unknown plugin type: " + pluginConfig.pluginType);
            }

            std::string pluginError;
            if (!plugin->configure(config, pluginError)) {
                error = pluginError;
                return common::WrapError(error, "configure plugin " + pluginConfig.pluginType + " failed: ");
            }
            plugins_[pluginConfig.pluginId]     = std::move(plugin);
            pluginTypes_[pluginConfig.pluginId] = pluginConfig.pluginType;
            PUPPET_LOG(INFO, "plugin_configured", "retargeting_pipeline", "configure")
                << " plugin_id=" << pluginConfig.pluginId << " plugin_type=" << pluginConfig.pluginType;
        }

        common::ClearError(error);
        return true;
    }

    bool RetargetingPipeline::requiresRobotState(const routing::GroupRoutingPlan& plan) const {
        const auto it = pluginTypes_.find(plan.pluginId);
        if (it == pluginTypes_.end()) {
            return false;
        }
        const std::string& pluginType = it->second;
        if (pluginType == "single_chain_ik" && plan.controlSemantics == "cartesian_delta") {
            return true;
        }
        if (pluginType == "single_chain_ik" && plan.controlSemantics == "cartesian_velocity") {
            return true;
        }
        const auto pluginIt = plugins_.find(plan.pluginId);
        if (pluginIt == plugins_.end()) {
            return false;
        }
        return pluginIt->second->requiresRobotState();
    }

    bool RetargetingPipeline::run(const routing::GroupRoutingPlan& plan, const model::PrimitiveFrame& frame,
                                  model::GroupControlIntent* output, std::string& error) const {
        const auto pluginIt = plugins_.find(plan.pluginId);
        if (pluginIt == plugins_.end()) {
            return common::Fail(error, "plugin not found: " + plan.pluginId);
        }

        const auto typeIt = pluginTypes_.find(plan.pluginId);
        if (typeIt == pluginTypes_.end()) {
            return common::Fail(error, "plugin type not found: " + plan.pluginId);
        }

        const std::string& pluginType = typeIt->second;
        PUPPET_VLOG(2, "plugin_dispatch", "retargeting_pipeline", "run")
            << " plugin_id=" << plan.pluginId << " plugin_type=" << pluginType << " body_group=" << plan.bodyGroup
            << " control_semantics=" << plan.controlSemantics;

        if (!supportsGroup(pluginType, plan.bodyGroup)) {
            return common::Fail(error,
                                "plugin " + plan.pluginId + " type " + pluginType + " does not support body_group: " + plan.bodyGroup);
        }

        if (!supportsSemantics(pluginType, plan.controlSemantics)) {
            return common::Fail(
                error, "plugin " + plan.pluginId + " type " + pluginType + " does not support control_semantics: " + plan.controlSemantics);
        }

        return pluginIt->second->process(frame, plan.bodyGroup, output, error);
    }

}  // namespace puppet::retargeting
