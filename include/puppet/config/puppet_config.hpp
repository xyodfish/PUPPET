#pragma once

#include <string>

#include "puppet/runtime/embosa_runtime_config.hpp"
#include "puppet/runtime/runtime_config.hpp"

namespace puppet::runtime {

    struct PuppetConfig {
        std::string configPath;
        RuntimeConfig runtime;
        EmbosaRuntimeConfig embosaRuntime;
    };

    class PuppetConfigLoader {
       public:
        static bool loadFromYamlFile(const std::string& path, PuppetConfig& config, std::string& error);
    };

}  // namespace puppet::runtime
