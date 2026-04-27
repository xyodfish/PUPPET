#pragma once

#include <string>

#include "puppet/runtime/runtime_config.hpp"
#include "puppet/transport/embosa/embosa_runtime_config.hpp"
#include "puppet/transport/tcp/tcp_runtime_config.hpp"
#include "puppet/transport/udp/udp_runtime_config.hpp"
#include "puppet/transport/zmq/zmq_runtime_config.hpp"

namespace puppet::runtime {

    struct PuppetConfig {
        std::string configPath;
        std::string runtimeChannelType = "embosa";
        RuntimeConfig runtime;
        EmbosaRuntimeConfig embosaRuntime;
        ZmqRuntimeConfig zmqRuntime;
        TcpRuntimeConfig tcpRuntime;
        UdpRuntimeConfig udpRuntime;
    };

    class PuppetConfigLoader {
       public:
        static bool loadFromYamlFile(const std::string& path, PuppetConfig& config, std::string& error);
    };

}  // namespace puppet::runtime
