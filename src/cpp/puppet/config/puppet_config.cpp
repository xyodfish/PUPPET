#include "puppet/config/puppet_config.hpp"

#include <yaml-cpp/yaml.h>

namespace puppet::runtime {

    bool PuppetConfigLoader::loadFromYamlFile(const std::string& path, PuppetConfig& config, std::string& error) {
        config.configPath         = path;
        config.runtimeChannelType = "embosa";

        try {
            const YAML::Node root = YAML::LoadFile(path);
            if (root["runtime_channel"] && root["runtime_channel"]["type"]) {
                config.runtimeChannelType = root["runtime_channel"]["type"].as<std::string>();
            } else if (root["channel"] && root["channel"]["type"]) {
                config.runtimeChannelType = root["channel"]["type"].as<std::string>();
            }
        } catch (const std::exception& ex) {
            error = "load channel config failed: " + std::string(ex.what());
            return false;
        }

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

        std::string zmqError;
        if (!loadZmqRuntimeConfig(path, config.zmqRuntime, zmqError)) {
            error = "load zmq runtime config failed: " + zmqError;
            return false;
        }

        error.clear();
        return true;
    }

}  // namespace puppet::runtime
