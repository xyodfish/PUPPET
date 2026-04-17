#pragma once

#include <string>
#include <vector>

namespace puppet::runtime {

    struct EmbosaRuntimeConfig {
        struct EndpointConfig {
            std::string key;
            std::string topicName;
            bool enabled = true;
        };

        std::string embosaNodeName           = "puppet_teleop_runtime";
        std::string inputTopicName           = "puppet_demo/primitive_frame";
        std::string outputControlIntentTopic = "puppet_demo/control_intent";
        int idleSleepMs                      = 2;
        std::vector<EndpointConfig> inputEndpoints{
            EndpointConfig{"primitive_frame", "puppet_demo/primitive_frame", true},
        };
        std::vector<EndpointConfig> outputEndpoints{
            EndpointConfig{"control_intent", "puppet_demo/control_intent", true},
        };
    };

    bool loadEmbosaRuntimeConfig(const std::string& runtimeYaml, EmbosaRuntimeConfig& cfg, std::string& error);

}  // namespace puppet::runtime
