#include "puppet/runtime/runtime_config.hpp"

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

    }  // namespace

    bool RuntimeConfigLoader::loadFromYamlFile(const std::string& path, RuntimeConfig* config, std::string* error) {
        if (config == nullptr) {
            if (error != nullptr) {
                *error = "config is null";
            }
            return false;
        }

        try {
            const YAML::Node yaml = YAML::LoadFile(path);

            config->loopHz           = readOr<int32_t>(yaml, "loop_hz", 100);
            config->enableMockSource = readOr<bool>(yaml, "enable_mock_source", true);
            config->mockHumanBodies.clear();
            if (yaml["mock_human_bodies"]) {
                config->mockHumanBodies = yaml["mock_human_bodies"].as<std::vector<std::string>>();
            } else {
                config->mockHumanBodies = {"pelvis",     "spine3",      "left_hip",   "right_hip",     "left_knee",
                                           "right_knee", "left_foot",   "right_foot", "left_shoulder", "right_shoulder",
                                           "left_elbow", "right_elbow", "left_wrist", "right_wrist"};
            }

            config->sources.clear();
            if (yaml["sources"]) {
                for (const auto& sourceNode : yaml["sources"]) {
                    SourceConfig source;
                    source.sourceId           = readOr<std::string>(sourceNode, "source_id", "");
                    source.sourceType         = readOr<std::string>(sourceNode, "source_type", "external");
                    source.topicName          = readOr<std::string>(sourceNode, "topic", source.sourceId);
                    source.freshnessTimeoutMs = readOr<int32_t>(sourceNode, "freshness_timeout_ms", 200);
                    source.historySize        = readOr<int32_t>(sourceNode, "history_size", 5);
                    if (!source.sourceId.empty()) {
                        config->sources.push_back(std::move(source));
                    }
                }
            }

            config->groupRouting.clear();
            if (yaml["group_routing"]) {
                for (const auto& routeNode : yaml["group_routing"]) {
                    GroupRoutingConfig route;
                    route.bodyGroup     = readOr<std::string>(routeNode, "body_group", "");
                    route.ownerSourceId = readOr<std::string>(routeNode, "owner_source", "");
                    route.mode          = readOr<std::string>(routeNode, "mode", "direct");
                    route.pipelineId    = readOr<std::string>(routeNode, "pipeline", "direct_pipeline");
                    route.backendId     = readOr<std::string>(routeNode, "backend", "direct_backend");
                    if (!route.bodyGroup.empty()) {
                        config->groupRouting.push_back(std::move(route));
                    }
                }
            }

            config->pipelines.clear();
            config->pipelineMap.clear();
            if (yaml["pipelines"]) {
                for (const auto& pipelineNode : yaml["pipelines"]) {
                    PipelineConfig pipeline;
                    pipeline.pipelineId = readOr<std::string>(pipelineNode, "pipeline_id", "");
                    pipeline.pluginType = readOr<std::string>(pipelineNode, "plugin", "direct_pass");
                    if (!pipeline.pipelineId.empty()) {
                        config->pipelineMap[pipeline.pipelineId] = pipeline;
                        config->pipelines.push_back(std::move(pipeline));
                    }
                }
            }

            config->backends.clear();
            config->backendMap.clear();
            if (yaml["backends"]) {
                for (const auto& backendNode : yaml["backends"]) {
                    BackendConfig backend;
                    backend.backendId   = readOr<std::string>(backendNode, "backend_id", "");
                    backend.backendType = readOr<std::string>(backendNode, "type", "direct_mapping");
                    if (!backend.backendId.empty()) {
                        config->backendMap[backend.backendId] = backend;
                        config->backends.push_back(std::move(backend));
                    }
                }
            }

            if (yaml["gmr_plugin"]) {
                const auto gmrNode              = yaml["gmr_plugin"];
                config->gmr.enabled             = readOr<bool>(gmrNode, "enabled", false);
                config->gmr.backendName         = readOr<std::string>(gmrNode, "backend", "pin_ik");
                config->gmr.robotModelPath      = readOr<std::string>(gmrNode, "robot_model_path", "");
                config->gmr.robotModelXmlPath   = readOr<std::string>(gmrNode, "robot_model_xml_path", "");
                config->gmr.ikConfigPath        = readOr<std::string>(gmrNode, "ik_config_path", "");
                config->gmr.actualHumanHeight   = readOr<double>(gmrNode, "actual_human_height", 1.75);
                config->gmr.damping             = readOr<double>(gmrNode, "damping", 0.5);
                config->gmr.maxIterations       = readOr<int32_t>(gmrNode, "max_iterations", 20);
                config->gmr.useVelocityLimit    = readOr<bool>(gmrNode, "use_velocity_limit", false);
                config->gmr.integrationTimestep = readOr<double>(gmrNode, "integration_timestep", 0.01);
                config->gmr.progressThreshold   = readOr<double>(gmrNode, "progress_threshold", 1e-3);
            }

            return true;
        } catch (const std::exception& ex) {
            if (error != nullptr) {
                *error = ex.what();
            }
            return false;
        }
    }

}  // namespace puppet::runtime
