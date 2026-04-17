#include "puppet/runtime/embosa_runtime_config.hpp"

#include <utility>

#include <yaml-cpp/yaml.h>

namespace puppet::runtime {

    namespace embosa_config_detail {
        std::vector<EmbosaRuntimeConfig::EndpointConfig> readEndpointConfigs(const YAML::Node& node, const std::string& fieldName) {
            std::vector<EmbosaRuntimeConfig::EndpointConfig> output;
            if (!node[fieldName]) {
                return output;
            }
            for (const auto& endpointNode : node[fieldName]) {
                EmbosaRuntimeConfig::EndpointConfig endpoint;
                if (endpointNode["key"]) {
                    endpoint.key = endpointNode["key"].as<std::string>();
                }
                if (endpointNode["topic"]) {
                    endpoint.topicName = endpointNode["topic"].as<std::string>();
                }
                if (endpointNode["enabled"]) {
                    endpoint.enabled = endpointNode["enabled"].as<bool>();
                }
                if (!endpoint.key.empty() && !endpoint.topicName.empty()) {
                    output.push_back(std::move(endpoint));
                }
            }
            return output;
        }
    }  // namespace embosa_config_detail

    bool loadEmbosaRuntimeConfig(const std::string& runtimeYaml, EmbosaRuntimeConfig& cfg, std::string& error) {
        try {
            YAML::Node root = YAML::LoadFile(runtimeYaml);
            if (root["embosa_runtime"]) {
                auto node = root["embosa_runtime"];
                if (node["node_name"]) {
                    cfg.embosaNodeName = node["node_name"].as<std::string>();
                }
                if (node["input_topic_name"]) {
                    cfg.inputTopicName = node["input_topic_name"].as<std::string>();
                } else if (node["topic_name"]) {
                    cfg.inputTopicName = node["topic_name"].as<std::string>();
                }
                if (node["output_control_intent_topic"]) {
                    cfg.outputControlIntentTopic = node["output_control_intent_topic"].as<std::string>();
                }
                if (node["idle_sleep_ms"]) {
                    cfg.idleSleepMs = node["idle_sleep_ms"].as<int>();
                }

                cfg.inputEndpoints = {
                    EmbosaRuntimeConfig::EndpointConfig{"primitive_frame", cfg.inputTopicName, true},
                };
                cfg.outputEndpoints = {
                    EmbosaRuntimeConfig::EndpointConfig{"control_intent", cfg.outputControlIntentTopic, true},
                };
                if (node["endpoints"]) {
                    const auto configuredInputs  = embosa_config_detail::readEndpointConfigs(node["endpoints"], "inputs");
                    const auto configuredOutputs = embosa_config_detail::readEndpointConfigs(node["endpoints"], "outputs");
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
