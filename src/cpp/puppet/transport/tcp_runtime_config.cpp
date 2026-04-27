#include "puppet/transport/tcp_runtime_config.hpp"

#include <filesystem>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace puppet::runtime::tcp_runtime_config_detail {

    std::string resolveYamlPath(const std::string& baseYamlPath, const std::string& maybeRelativePath) {
        namespace fs = std::filesystem;
        const fs::path rawPath(maybeRelativePath);
        if (rawPath.is_absolute()) {
            return rawPath.lexically_normal().string();
        }
        const fs::path basePath(baseYamlPath);
        return (basePath.parent_path() / rawPath).lexically_normal().string();
    }

    std::vector<TcpRuntimeConfig::EndpointConfig> readEndpointConfigs(const YAML::Node& node, const std::string& fieldName) {
        std::vector<TcpRuntimeConfig::EndpointConfig> output;
        if (!node[fieldName]) {
            return output;
        }
        for (const auto& endpointNode : node[fieldName]) {
            TcpRuntimeConfig::EndpointConfig endpoint;
            if (endpointNode["key"]) {
                endpoint.key = endpointNode["key"].as<std::string>();
            }
            if (endpointNode["host"]) {
                endpoint.host = endpointNode["host"].as<std::string>();
            }
            if (endpointNode["port"]) {
                endpoint.port = endpointNode["port"].as<uint16_t>();
            }
            if (endpointNode["enabled"]) {
                endpoint.enabled = endpointNode["enabled"].as<bool>();
            }
            if (!endpoint.key.empty() && !endpoint.host.empty() && endpoint.port > 0) {
                output.push_back(std::move(endpoint));
            }
        }
        return output;
    }

}  // namespace puppet::runtime::tcp_runtime_config_detail

namespace puppet::runtime {

    bool loadTcpRuntimeConfig(const std::string& runtimeYaml, TcpRuntimeConfig& cfg, std::string& error) {
        try {
            YAML::Node root = YAML::LoadFile(runtimeYaml);
            YAML::Node node = root["tcp_runtime"];
            if (root["module_configs"] && root["module_configs"]["tcp_runtime"]) {
                const std::string modulePath = root["module_configs"]["tcp_runtime"].as<std::string>();
                if (!modulePath.empty()) {
                    const YAML::Node moduleRoot = YAML::LoadFile(tcp_runtime_config_detail::resolveYamlPath(runtimeYaml, modulePath));
                    if (moduleRoot["tcp_runtime"]) {
                        node = moduleRoot["tcp_runtime"];
                    } else {
                        node = moduleRoot;
                    }
                }
            }

            if (node) {
                if (node["use_length_prefix"]) {
                    cfg.useLengthPrefix = node["use_length_prefix"].as<bool>();
                }
                if (node["idle_sleep_ms"]) {
                    cfg.idleSleepMs = node["idle_sleep_ms"].as<int>();
                }
                if (node["recv_buffer_size"]) {
                    cfg.recvBufferSize = node["recv_buffer_size"].as<int>();
                }
                if (node["input_host"]) {
                    cfg.inputHost = node["input_host"].as<std::string>();
                }
                if (node["input_port"]) {
                    cfg.inputPort = node["input_port"].as<uint16_t>();
                }
                if (node["robot_state_input_host"]) {
                    cfg.robotStateInputHost = node["robot_state_input_host"].as<std::string>();
                }
                if (node["robot_state_input_port"]) {
                    cfg.robotStateInputPort = node["robot_state_input_port"].as<uint16_t>();
                }
                if (node["output_control_intent_host"]) {
                    cfg.outputControlHost = node["output_control_intent_host"].as<std::string>();
                }
                if (node["output_control_intent_port"]) {
                    cfg.outputControlIntentPort = node["output_control_intent_port"].as<uint16_t>();
                }

                cfg.inputEndpoints = {
                    TcpRuntimeConfig::EndpointConfig{"primitive_frame", cfg.inputHost, cfg.inputPort, true},
                    TcpRuntimeConfig::EndpointConfig{"robot_state_frame", cfg.robotStateInputHost, cfg.robotStateInputPort, false},
                };
                cfg.outputEndpoints = {
                    TcpRuntimeConfig::EndpointConfig{"control_intent", cfg.outputControlHost, cfg.outputControlIntentPort, true},
                };

                if (node["endpoints"]) {
                    const auto configuredInputs  = tcp_runtime_config_detail::readEndpointConfigs(node["endpoints"], "inputs");
                    const auto configuredOutputs = tcp_runtime_config_detail::readEndpointConfigs(node["endpoints"], "outputs");
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
