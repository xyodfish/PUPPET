#pragma once

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "puppet/primitive/primitive_types.hpp"

namespace puppet::transport {

    struct DeviceOutputChannelConfig {
        std::string nodeName;
        std::string topicName;
        std::string outputEndpoint;
        YAML::Node channelNode;
    };

    class IDeviceOutputChannel {
       public:
        virtual ~IDeviceOutputChannel() = default;

        virtual bool initialize(const DeviceOutputChannelConfig& config, std::string& error) = 0;

        virtual bool publish(const model::PrimitiveFrame& frame, std::string& error) = 0;

        virtual bool ok() const = 0;

        virtual void shutdown() = 0;
    };

    std::unique_ptr<IDeviceOutputChannel> createDeviceOutputChannel(const std::string& channelType, std::string& error);

}  // namespace puppet::transport
