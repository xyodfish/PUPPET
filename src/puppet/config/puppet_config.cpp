#include "puppet/config/puppet_config.hpp"

namespace puppet::runtime {

    bool PuppetConfigLoader::loadFromYamlFile(const std::string& path, PuppetConfig& config, std::string& error) {
        config.configPath = path;

        std::string runtimeError;
        if (!RuntimeConfigLoader::loadFromYamlFile(path, config.runtime, runtimeError)) {
            error = "load runtime config failed: " + runtimeError;
            return false;
        }

        std::string embosaError;
        if (!loadEmbosaRuntimeConfig(path, config.embosaRuntime, embosaError)) {
            error = "load embosa runtime config failed: " + embosaError;
            return false;
        }

        error.clear();
        return true;
    }

}  // namespace puppet::runtime
