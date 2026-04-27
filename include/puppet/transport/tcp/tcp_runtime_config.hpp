#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace puppet::runtime {

    struct TcpRuntimeConfig {
        struct EndpointConfig {
            std::string key;
            std::string host;
            uint16_t port = 0;
            bool enabled  = true;
        };

        bool useLengthPrefix = false;
        int idleSleepMs      = 2;
        int recvBufferSize   = 4096;

        std::string inputHost            = "127.0.0.1";
        uint16_t inputPort               = 16661;
        std::string robotStateInputHost  = "127.0.0.1";
        uint16_t robotStateInputPort     = 16662;
        std::string outputControlHost    = "127.0.0.1";
        uint16_t outputControlIntentPort = 16663;

        std::vector<EndpointConfig> inputEndpoints{
            EndpointConfig{"primitive_frame", "127.0.0.1", 16661, true},
            EndpointConfig{"robot_state_frame", "127.0.0.1", 16662, false},
        };
        std::vector<EndpointConfig> outputEndpoints{
            EndpointConfig{"control_intent", "127.0.0.1", 16663, true},
        };
    };

    bool loadTcpRuntimeConfig(const std::string& runtimeYaml, TcpRuntimeConfig& cfg, std::string& error);

}  // namespace puppet::runtime
