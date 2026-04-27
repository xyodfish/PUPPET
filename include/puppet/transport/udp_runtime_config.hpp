#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace puppet::runtime {

    struct UdpRuntimeConfig {
        struct InputEndpointConfig {
            std::string key;
            std::string bindHost = "0.0.0.0";
            uint16_t bindPort    = 0;
            bool enabled         = true;
        };

        struct OutputEndpointConfig {
            std::string key;
            std::string remoteHost = "127.0.0.1";
            uint16_t remotePort    = 0;
            bool enabled           = true;
        };

        int idleSleepMs    = 2;
        int recvBufferSize = 65536;

        std::string inputBindHost           = "0.0.0.0";
        uint16_t inputBindPort              = 16661;
        std::string robotStateInputBindHost = "0.0.0.0";
        uint16_t robotStateInputBindPort    = 16662;
        std::string outputRemoteHost        = "127.0.0.1";
        uint16_t outputControlIntentPort    = 16663;

        std::vector<InputEndpointConfig> inputEndpoints{
            InputEndpointConfig{"primitive_frame", "0.0.0.0", 16661, true},
            InputEndpointConfig{"robot_state_frame", "0.0.0.0", 16662, false},
        };
        std::vector<OutputEndpointConfig> outputEndpoints{
            OutputEndpointConfig{"control_intent", "127.0.0.1", 16663, true},
        };
    };

    bool loadUdpRuntimeConfig(const std::string& runtimeYaml, UdpRuntimeConfig& cfg, std::string& error);

}  // namespace puppet::runtime
