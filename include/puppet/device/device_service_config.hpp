#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace puppet::device {

    struct DeviceServiceConfig {
        std::string nodeName       = "puppet_device_service";
        std::string topicName      = "puppet_demo/primitive_frame";
        std::string sourceId       = "device_service";
        std::string frameId        = "world";
        std::string channelType    = "embosa";
        std::string deviceType     = "static_file_replay";
        int loopHz                 = 50;
        std::string outputEndpoint = "tcp://127.0.0.1:16661";

        YAML::Node channelNode;
        YAML::Node deviceNode;
    };

    bool loadDeviceServiceConfig(const std::string& path, DeviceServiceConfig* config, std::string* error);

}  // namespace puppet::device
