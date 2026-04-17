#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace puppet::runtime {

    struct SourceConfig {
        std::string sourceId;
        std::string sourceType;
        std::string topicName;
        int32_t freshnessTimeoutMs = 200;
        int32_t historySize        = 5;
    };

    struct GroupRoutingConfig {
        std::string bodyGroup;
        std::string ownerSourceId;
        std::string mode;
        std::string pipelineId;
        std::string backendId;
    };

    struct PipelineConfig {
        std::string pipelineId;
        std::string pluginType;
    };

    struct BackendConfig {
        std::string backendId;
        std::string backendType;
    };

    struct GmrPluginConfig {
        bool enabled            = false;
        std::string backendName = "pin_ik";
        std::string robotModelPath;
        std::string robotModelXmlPath;
        std::string ikConfigPath;
        double actualHumanHeight   = 1.75;
        double damping             = 0.5;
        int32_t maxIterations      = 20;
        bool useVelocityLimit      = false;
        double integrationTimestep = 0.01;
        double progressThreshold   = 1e-3;
    };

    struct RuntimeConfig {
        int32_t loopHz        = 100;
        bool enableMockSource = true;
        std::vector<std::string> mockHumanBodies;

        std::vector<SourceConfig> sources;
        std::vector<GroupRoutingConfig> groupRouting;
        std::vector<PipelineConfig> pipelines;
        std::vector<BackendConfig> backends;
        GmrPluginConfig gmr;

        std::unordered_map<std::string, PipelineConfig> pipelineMap;
        std::unordered_map<std::string, BackendConfig> backendMap;
    };

    class RuntimeConfigLoader {
       public:
        static bool loadFromYamlFile(const std::string& path, RuntimeConfig* config, std::string* error);
    };

}  // namespace puppet::runtime
