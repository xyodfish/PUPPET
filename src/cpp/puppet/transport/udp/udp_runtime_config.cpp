#include "puppet/transport/udp/udp_runtime_config.hpp"

#include <filesystem>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace puppet::runtime::udp_runtime_config_detail {

    std::string resolveYamlPath(const std::string& baseYamlPath, const std::string& maybeRelativePath) {
        namespace fs = std::filesystem;
        const fs::path rawPath(maybeRelativePath);
        if (rawPath.is_absolute()) {
            return rawPath.lexically_normal().string();
        }
        const fs::path basePath(baseYamlPath);
        return (basePath.parent_path() / rawPath).lexically_normal().string();
    }

    std::vector<UdpRuntimeConfig::InputEndpointConfig> readInputEndpointConfigs(const YAML::Node& node, const std::string& fieldName) {
        std::vector<UdpRuntimeConfig::InputEndpointConfig> output;
        if (!node[fieldName]) {
            return output;
        }
        for (const auto& endpointNode : node[fieldName]) {
            UdpRuntimeConfig::InputEndpointConfig endpoint;
            if (endpointNode["key"]) {
                endpoint.key = endpointNode["key"].as<std::string>();
            }
            if (endpointNode["bind_host"]) {
                endpoint.bindHost = endpointNode["bind_host"].as<std::string>();
            }
            if (endpointNode["bind_port"]) {
                endpoint.bindPort = endpointNode["bind_port"].as<uint16_t>();
            }
            if (endpointNode["enabled"]) {
                endpoint.enabled = endpointNode["enabled"].as<bool>();
            }
            if (!endpoint.key.empty() && endpoint.bindPort > 0) {
                output.push_back(std::move(endpoint));
            }
        }
        return output;
    }

    std::vector<UdpRuntimeConfig::OutputEndpointConfig> readOutputEndpointConfigs(const YAML::Node& node, const std::string& fieldName) {
        std::vector<UdpRuntimeConfig::OutputEndpointConfig> output;
        if (!node[fieldName]) {
            return output;
        }
        for (const auto& endpointNode : node[fieldName]) {
            UdpRuntimeConfig::OutputEndpointConfig endpoint;
            if (endpointNode["key"]) {
                endpoint.key = endpointNode["key"].as<std::string>();
            }
            if (endpointNode["remote_host"]) {
                endpoint.remoteHost = endpointNode["remote_host"].as<std::string>();
            }
            if (endpointNode["remote_port"]) {
                endpoint.remotePort = endpointNode["remote_port"].as<uint16_t>();
            }
            if (endpointNode["enabled"]) {
                endpoint.enabled = endpointNode["enabled"].as<bool>();
            }
            if (!endpoint.key.empty() && !endpoint.remoteHost.empty() && endpoint.remotePort > 0) {
                output.push_back(std::move(endpoint));
            }
        }
        return output;
    }

}  // namespace puppet::runtime::udp_runtime_config_detail

namespace puppet::runtime {

    bool loadUdpRuntimeConfig(const std::string& runtimeYaml, UdpRuntimeConfig& cfg, std::string& error) {
        try {
            YAML::Node root = YAML::LoadFile(runtimeYaml);
            YAML::Node node = root["udp_runtime"];
            if (root["module_configs"] && root["module_configs"]["udp_runtime"]) {
                const std::string modulePath = root["module_configs"]["udp_runtime"].as<std::string>();
                if (!modulePath.empty()) {
                    const YAML::Node moduleRoot = YAML::LoadFile(udp_runtime_config_detail::resolveYamlPath(runtimeYaml, modulePath));
                    if (moduleRoot["udp_runtime"]) {
                        node = moduleRoot["udp_runtime"];
                    } else {
                        node = moduleRoot;
                    }
                }
            }

            if (node) {
                if (node["idle_sleep_ms"]) {
                    cfg.idleSleepMs = node["idle_sleep_ms"].as<int>();
                }
                if (node["recv_buffer_size"]) {
                    cfg.recvBufferSize = node["recv_buffer_size"].as<int>();
                }
                if (node["input_bind_host"]) {
                    cfg.inputBindHost = node["input_bind_host"].as<std::string>();
                }
                if (node["input_bind_port"]) {
                    cfg.inputBindPort = node["input_bind_port"].as<uint16_t>();
                }
                if (node["robot_state_input_bind_host"]) {
                    cfg.robotStateInputBindHost = node["robot_state_input_bind_host"].as<std::string>();
                }
                if (node["robot_state_input_bind_port"]) {
                    cfg.robotStateInputBindPort = node["robot_state_input_bind_port"].as<uint16_t>();
                }
                if (node["output_remote_host"]) {
                    cfg.outputRemoteHost = node["output_remote_host"].as<std::string>();
                }
                if (node["output_control_intent_port"]) {
                    cfg.outputControlIntentPort = node["output_control_intent_port"].as<uint16_t>();
                }

                cfg.inputEndpoints = {
                    UdpRuntimeConfig::InputEndpointConfig{"primitive_frame", cfg.inputBindHost, cfg.inputBindPort, true},
                    UdpRuntimeConfig::InputEndpointConfig{"robot_state_frame", cfg.robotStateInputBindHost, cfg.robotStateInputBindPort,
                                                          false},
                };
                cfg.outputEndpoints = {
                    UdpRuntimeConfig::OutputEndpointConfig{"control_intent", cfg.outputRemoteHost, cfg.outputControlIntentPort, true},
                };

                if (node["endpoints"]) {
                    const auto configuredInputs  = udp_runtime_config_detail::readInputEndpointConfigs(node["endpoints"], "inputs");
                    const auto configuredOutputs = udp_runtime_config_detail::readOutputEndpointConfigs(node["endpoints"], "outputs");
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
