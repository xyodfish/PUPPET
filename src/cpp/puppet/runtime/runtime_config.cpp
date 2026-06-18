#include "puppet/runtime/runtime_config.hpp"

#include <filesystem>

#include <yaml-cpp/yaml.h>

namespace puppet::runtime {
    namespace {

        template <typename T>
        T readOr(const YAML::Node& node, const char* key, const T& fallback) {
            if (!node[key]) {
                return fallback;
            }
            return node[key].as<T>();
        }

        std::string readStringOr(const YAML::Node& node, const char* key, const char* legacyKey, const std::string& fallback) {
            if (node[key]) {
                return node[key].as<std::string>();
            }
            if (legacyKey != nullptr && node[legacyKey]) {
                return node[legacyKey].as<std::string>();
            }
            return fallback;
        }

        std::string resolveYamlPath(const std::string& baseYamlPath, const std::string& maybeRelativePath) {
            namespace fs = std::filesystem;
            const fs::path rawPath(maybeRelativePath);
            if (rawPath.is_absolute()) {
                return rawPath.lexically_normal().string();
            }
            const fs::path basePath(baseYamlPath);
            const fs::path parent = basePath.parent_path();
            return (parent / rawPath).lexically_normal().string();
        }

        YAML::Node loadSectionNode(const YAML::Node& root, const std::string& sectionName, const std::string& baseYamlPath) {
            if (root["module_configs"] && root["module_configs"][sectionName]) {
                const std::string modulePath = root["module_configs"][sectionName].as<std::string>();
                if (!modulePath.empty()) {
                    const YAML::Node moduleRoot = YAML::LoadFile(resolveYamlPath(baseYamlPath, modulePath));
                    if (moduleRoot[sectionName]) {
                        return moduleRoot[sectionName];
                    }
                    return moduleRoot;
                }
            }
            return root[sectionName];
        }

        void parseActivePlugins(const YAML::Node& routeNode, std::vector<PluginConfig>* activePlugins) {
            if (activePlugins == nullptr) {
                return;
            }

            activePlugins->clear();

            const YAML::Node pluginsNode = routeNode["active_plugins"];
            if (!pluginsNode || !pluginsNode.IsSequence()) {
                return;
            }

            auto appendPlugin = [&](const YAML::Node& pluginNode) {
                PluginConfig pluginCfg;
                pluginCfg.pluginId   = readStringOr(pluginNode, "plugin_id", "pipeline_id", "");
                pluginCfg.pluginType = readOr<std::string>(pluginNode, "type", "direct_pass");
                pluginCfg.enabled    = readOr<bool>(pluginNode, "enabled", true);
                activePlugins->push_back(std::move(pluginCfg));
            };

            if (pluginsNode.IsSequence()) {
                for (const auto& pluginNode : pluginsNode) {
                    appendPlugin(pluginNode);
                }
            }
        }

    }  // namespace

    bool RuntimeConfigLoader::loadFromYamlFile(const std::string& path, RuntimeConfig& config, std::string& error) {
        try {
            const YAML::Node yaml = YAML::LoadFile(path);

            config.loopHz           = readOr<int32_t>(yaml, "loop_hz", 100);
            config.enableMockSource = readOr<bool>(yaml, "enable_mock_source", true);
            config.mockHumanBodies.clear();
            if (yaml["mock_human_bodies"]) {
                config.mockHumanBodies = yaml["mock_human_bodies"].as<std::vector<std::string>>();
            } else {
                config.mockHumanBodies = {"pelvis",     "spine3",      "left_hip",   "right_hip",     "left_knee",
                                          "right_knee", "left_foot",   "right_foot", "left_shoulder", "right_shoulder",
                                          "left_elbow", "right_elbow", "left_wrist", "right_wrist"};
            }

            config.sources.clear();
            const YAML::Node sourcesNode = loadSectionNode(yaml, "sources", path);
            if (sourcesNode) {
                for (const auto& sourceNode : sourcesNode) {
                    SourceConfig source;
                    source.sourceId           = readOr<std::string>(sourceNode, "source_id", "");
                    source.sourceType         = readOr<std::string>(sourceNode, "source_type", "external");
                    source.topicName          = readOr<std::string>(sourceNode, "topic", source.sourceId);
                    source.freshnessTimeoutMs = readOr<int32_t>(sourceNode, "freshness_timeout_ms", 200);
                    source.historySize        = readOr<int32_t>(sourceNode, "history_size", 5);
                    if (!source.sourceId.empty()) {
                        config.sources.push_back(std::move(source));
                    }
                }
            }

            config.groupRouting.clear();
            const YAML::Node routingNode = loadSectionNode(yaml, "group_routing", path);
            if (routingNode) {
                for (const auto& routeNode : routingNode) {
                    GroupRoutingConfig route;
                    route.bodyGroup        = readOr<std::string>(routeNode, "body_group", "");
                    route.ownerSourceId    = readOr<std::string>(routeNode, "owner_source", "");
                    route.mode             = readOr<std::string>(routeNode, "mode", "direct");
                    route.pluginId         = readStringOr(routeNode, "plugin", "pipeline", "direct_plugin");
                    route.backendId        = readOr<std::string>(routeNode, "backend", "direct_backend");
                    route.controlSemantics = readOr<std::string>(routeNode, "control_semantics", "cartesian_absolute");
                    route.priority         = readOr<int32_t>(routeNode, "priority", 0);
                    route.enabled          = readOr<bool>(routeNode, "enabled", true);
                    parseActivePlugins(routeNode, &route.activedPlugins);
                    if (!route.bodyGroup.empty()) {
                        config.groupRouting.push_back(std::move(route));
                    }
                }
            }

            config.plugins.clear();
            config.pluginMap.clear();
            YAML::Node pluginsNode = loadSectionNode(yaml, "plugins", path);
            if (!pluginsNode) {
                pluginsNode = loadSectionNode(yaml, "pipelines", path);
            }
            if (pluginsNode) {
                for (const auto& pluginNode : pluginsNode) {
                    PluginConfig plugin;
                    plugin.pluginId   = readStringOr(pluginNode, "plugin_id", "pipeline_id", "");
                    plugin.pluginType = readOr<std::string>(pluginNode, "plugin", "direct_pass");
                    plugin.enabled    = readOr<bool>(pluginNode, "enabled", true);
                    if (!plugin.pluginId.empty()) {
                        config.pluginMap[plugin.pluginId] = plugin;
                        config.plugins.push_back(std::move(plugin));
                    }
                }
            }

            config.backends.clear();
            config.backendMap.clear();
            const YAML::Node backendsNode = loadSectionNode(yaml, "backends", path);
            if (backendsNode) {
                for (const auto& backendNode : backendsNode) {
                    BackendConfig backend;
                    backend.backendId   = readOr<std::string>(backendNode, "backend_id", "");
                    backend.backendType = readOr<std::string>(backendNode, "type", "direct_mapping");
                    if (!backend.backendId.empty()) {
                        config.backendMap[backend.backendId] = backend;
                        config.backends.push_back(std::move(backend));
                    }
                }
            }

            const YAML::Node gmrNode = loadSectionNode(yaml, "gmr_plugin", path);
            if (gmrNode) {
                config.gmr.enabled             = readOr<bool>(gmrNode, "enabled", false);
                config.gmr.backendName         = readOr<std::string>(gmrNode, "backend", "pin_ik");
                config.gmr.robotModelPath      = readOr<std::string>(gmrNode, "robot_model_path", "");
                config.gmr.robotModelXmlPath   = readOr<std::string>(gmrNode, "robot_model_xml_path", "");
                config.gmr.ikConfigPath        = readOr<std::string>(gmrNode, "ik_config_path", "");
                config.gmr.actualHumanHeight   = readOr<double>(gmrNode, "actual_human_height", 1.75);
                config.gmr.damping             = readOr<double>(gmrNode, "damping", 0.5);
                config.gmr.maxIterations       = readOr<int32_t>(gmrNode, "max_iterations", 20);
                config.gmr.useVelocityLimit    = readOr<bool>(gmrNode, "use_velocity_limit", false);
                config.gmr.integrationTimestep = readOr<double>(gmrNode, "integration_timestep", 0.01);
                config.gmr.progressThreshold   = readOr<double>(gmrNode, "progress_threshold", 1e-3);
            }

            config.singleChainIk.chains.clear();
            config.singleChainIk.chainMap.clear();
            const YAML::Node ikNode = loadSectionNode(yaml, "single_chain_ik", path);
            if (ikNode) {
                config.singleChainIk.enabled = readOr<bool>(ikNode, "enabled", false);
                if (ikNode["chains"]) {
                    for (const auto& chainNode : ikNode["chains"]) {
                        SingleChainIkChainConfig chain;
                        chain.bodyGroup = readOr<std::string>(chainNode, "body_group", "");
                        chain.eeEntity  = readOr<std::string>(chainNode, "ee_entity", "");
                        chain.urdfPath  = readOr<std::string>(chainNode, "urdf_path", "");
                        chain.baseLink  = readOr<std::string>(chainNode, "base_link", "");
                        chain.tipLink   = readOr<std::string>(chainNode, "tip_link", "");
                        if (chainNode["joint_names"]) {
                            chain.jointNames = chainNode["joint_names"].as<std::vector<std::string>>();
                        }
                        if (chainNode["default_posture"]) {
                            chain.defaultPosture = chainNode["default_posture"].as<std::vector<double>>();
                        }
                        if (chainNode["min_position_limit"]) {
                            chain.minPositionLimit = chainNode["min_position_limit"].as<std::vector<double>>();
                        }
                        if (chainNode["max_position_limit"]) {
                            chain.maxPositionLimit = chainNode["max_position_limit"].as<std::vector<double>>();
                        }
                        if (chainNode["position_gains"]) {
                            chain.positionGains = chainNode["position_gains"].as<std::vector<double>>();
                        }
                        if (chainNode["orientation_gains"]) {
                            chain.orientationGains = chainNode["orientation_gains"].as<std::vector<double>>();
                        }
                        chain.maxIterations             = readOr<int32_t>(chainNode, "max_iterations", 10000);
                        chain.timeoutSec                = readOr<double>(chainNode, "timeout_sec", 0.003);
                        chain.epsilon                   = readOr<double>(chainNode, "epsilon", 1e-5);
                        chain.randomRestart             = readOr<bool>(chainNode, "random_restart", false);
                        chain.perturbationMaxAttempts   = readOr<int32_t>(chainNode, "perturbation_max_attempts", 5);
                        chain.perturbationPositionRange = readOr<double>(chainNode, "perturbation_position_range", 0.01);
                        chain.perturbationAngleDegRange = readOr<double>(chainNode, "perturbation_angle_deg_range", 5.0);

                        if (!chain.bodyGroup.empty() && !chain.jointNames.empty() && !chain.urdfPath.empty() && !chain.baseLink.empty() &&
                            !chain.tipLink.empty()) {
                            config.singleChainIk.chainMap[chain.bodyGroup] = chain;
                            config.singleChainIk.chains.push_back(std::move(chain));
                        }
                    }
                }
            }

            const YAML::Node robotStateNode = loadSectionNode(yaml, "robot_state", path);
            if (robotStateNode) {
                config.robotState.freshnessTimeoutMs = readOr<int32_t>(robotStateNode, "freshness_timeout_ms", 200);
            }

            error.clear();
            return true;
        } catch (const std::exception& ex) {
            error = ex.what();
            return false;
        }
    }

}  // namespace puppet::runtime
