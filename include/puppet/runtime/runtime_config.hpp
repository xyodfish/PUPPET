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
        bool enabled = true;
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

    struct SingleChainIkChainConfig {
        std::string bodyGroup;
        std::string eeEntity;
        std::string urdfPath;
        std::string baseLink;
        std::string tipLink;
        std::vector<std::string> jointNames;
        std::vector<double> defaultPosture;
        std::vector<double> minPositionLimit;
        std::vector<double> maxPositionLimit;
        std::vector<double> positionGains{1.0, 1.0, 1.0};
        std::vector<double> orientationGains{1.0, 1.0, 1.0};
        int32_t maxIterations            = 10000;
        double timeoutSec                = 0.003;
        double epsilon                   = 1e-5;
        bool randomRestart               = false;
        int32_t perturbationMaxAttempts  = 5;
        double perturbationPositionRange = 0.01;
        double perturbationAngleDegRange = 5.0;
    };

    struct SingleChainIkConfig {
        bool enabled = false;
        std::vector<SingleChainIkChainConfig> chains;
        std::unordered_map<std::string, SingleChainIkChainConfig> chainMap;
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
        SingleChainIkConfig singleChainIk;

        std::unordered_map<std::string, PipelineConfig> pipelineMap;
        std::unordered_map<std::string, BackendConfig> backendMap;
    };

    class RuntimeConfigLoader {
       public:
        static bool loadFromYamlFile(const std::string& path, RuntimeConfig& config, std::string& error);
    };

}  // namespace puppet::runtime
