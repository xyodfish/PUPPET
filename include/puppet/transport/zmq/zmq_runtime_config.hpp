#pragma once

#include <string>
#include <vector>

namespace puppet::runtime {

    struct ZmqRuntimeConfig {
        struct EndpointConfig {
            std::string key;
            std::string endpoint;
            std::string topicName;
            bool enabled = true;
        };

        int ioThreads         = 1;
        int recvHighWaterMark = 200;
        int sendHighWaterMark = 200;
        int idleSleepMs       = 2;

        std::string inputEndpoint               = "tcp://127.0.0.1:16661";
        std::string inputTopicName              = "puppet_demo/primitive_frame";
        std::string robotStateInputEndpoint     = "tcp://127.0.0.1:16662";
        std::string robotStateInputTopicName    = "puppet_demo/robot_state";
        std::string outputControlIntentEndpoint = "tcp://127.0.0.1:16663";
        std::string outputControlIntentTopic    = "puppet_demo/control_intent";

        std::vector<EndpointConfig> inputEndpoints{
            EndpointConfig{"primitive_frame", "tcp://127.0.0.1:16661", "puppet_demo/primitive_frame", true},
            EndpointConfig{"robot_state_frame", "tcp://127.0.0.1:16662", "puppet_demo/robot_state", false},
        };
        std::vector<EndpointConfig> outputEndpoints{
            EndpointConfig{"control_intent", "tcp://127.0.0.1:16663", "puppet_demo/control_intent", true},
        };
    };

    bool loadZmqRuntimeConfig(const std::string& runtimeYaml, ZmqRuntimeConfig& cfg, std::string& error);

}  // namespace puppet::runtime
