#include "puppet/transport/zmq/zmq_runtime_config.hpp"

#include <filesystem>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace puppet::runtime::zmq_runtime_config_detail {

    std::string resolveYamlPath(const std::string& baseYamlPath, const std::string& maybeRelativePath) {
        namespace fs = std::filesystem;
        const fs::path rawPath(maybeRelativePath);
        if (rawPath.is_absolute()) {
            return rawPath.lexically_normal().string();
        }
        const fs::path basePath(baseYamlPath);
        return (basePath.parent_path() / rawPath).lexically_normal().string();
    }

    std::vector<ZmqRuntimeConfig::EndpointConfig> readEndpointConfigs(const YAML::Node& node, const std::string& fieldName) {
        std::vector<ZmqRuntimeConfig::EndpointConfig> output;
        if (!node[fieldName]) {
            return output;
        }

        for (const auto& endpointNode : node[fieldName]) {
            ZmqRuntimeConfig::EndpointConfig endpoint;
            if (endpointNode["key"]) {
                endpoint.key = endpointNode["key"].as<std::string>();
            }
            if (endpointNode["endpoint"]) {
                endpoint.endpoint = endpointNode["endpoint"].as<std::string>();
            }
            if (endpointNode["topic"]) {
                endpoint.topicName = endpointNode["topic"].as<std::string>();
            }
            if (endpointNode["enabled"]) {
                endpoint.enabled = endpointNode["enabled"].as<bool>();
            }
            if (!endpoint.key.empty() && !endpoint.endpoint.empty()) {
                output.push_back(std::move(endpoint));
            }
        }
        return output;
    }

}  // namespace puppet::runtime::zmq_runtime_config_detail

namespace puppet::runtime {

    bool loadZmqRuntimeConfig(const std::string& runtimeYaml, ZmqRuntimeConfig& cfg, std::string& error) {
        try {
            YAML::Node root = YAML::LoadFile(runtimeYaml);
            YAML::Node node = root["zmq_runtime"];
            if (root["module_configs"] && root["module_configs"]["zmq_runtime"]) {
                const std::string modulePath = root["module_configs"]["zmq_runtime"].as<std::string>();
                if (!modulePath.empty()) {
                    const YAML::Node moduleRoot = YAML::LoadFile(zmq_runtime_config_detail::resolveYamlPath(runtimeYaml, modulePath));
                    if (moduleRoot["zmq_runtime"]) {
                        node = moduleRoot["zmq_runtime"];
                    } else {
                        node = moduleRoot;
                    }
                }
            }

            if (node) {
                if (node["io_threads"]) {
                    cfg.ioThreads = node["io_threads"].as<int>();
                }
                if (node["recv_hwm"]) {
                    cfg.recvHighWaterMark = node["recv_hwm"].as<int>();
                }
                if (node["send_hwm"]) {
                    cfg.sendHighWaterMark = node["send_hwm"].as<int>();
                }
                if (node["idle_sleep_ms"]) {
                    cfg.idleSleepMs = node["idle_sleep_ms"].as<int>();
                }
                if (node["input_endpoint"]) {
                    cfg.inputEndpoint = node["input_endpoint"].as<std::string>();
                }
                if (node["input_topic_name"]) {
                    cfg.inputTopicName = node["input_topic_name"].as<std::string>();
                }
                if (node["robot_state_input_endpoint"]) {
                    cfg.robotStateInputEndpoint = node["robot_state_input_endpoint"].as<std::string>();
                }
                if (node["robot_state_input_topic_name"]) {
                    cfg.robotStateInputTopicName = node["robot_state_input_topic_name"].as<std::string>();
                }
                if (node["output_control_intent_endpoint"]) {
                    cfg.outputControlIntentEndpoint = node["output_control_intent_endpoint"].as<std::string>();
                }
                if (node["output_control_intent_topic"]) {
                    cfg.outputControlIntentTopic = node["output_control_intent_topic"].as<std::string>();
                }

                cfg.inputEndpoints = {
                    ZmqRuntimeConfig::EndpointConfig{"primitive_frame", cfg.inputEndpoint, cfg.inputTopicName, true},
                    ZmqRuntimeConfig::EndpointConfig{"robot_state_frame", cfg.robotStateInputEndpoint, cfg.robotStateInputTopicName, false},
                };
                cfg.outputEndpoints = {
                    ZmqRuntimeConfig::EndpointConfig{"control_intent", cfg.outputControlIntentEndpoint, cfg.outputControlIntentTopic, true},
                };

                if (node["endpoints"]) {
                    const auto configuredInputs  = zmq_runtime_config_detail::readEndpointConfigs(node["endpoints"], "inputs");
                    const auto configuredOutputs = zmq_runtime_config_detail::readEndpointConfigs(node["endpoints"], "outputs");
                    if (!configuredInputs.empty()) {
                        cfg.inputEndpoints = configuredInputs;
                    }
                    if (!configuredOutputs.empty()) {
                        cfg.outputEndpoints = configuredOutputs;
                    }
                }
            }

            error.clear();
            return true;
        } catch (const std::exception& ex) {
            error = ex.what();
            return false;
        }
    }

}  // namespace puppet::runtime
