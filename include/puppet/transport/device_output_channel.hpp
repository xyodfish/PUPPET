#pragma once

#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "puppet/primitive/primitive_types.hpp"

namespace puppet::transport {

    class IDeviceOutputChannel {
       public:
        virtual ~IDeviceOutputChannel() = default;

        virtual bool initialize(const std::string& nodeName, const std::string& topicName, const std::string& outputEndpoint,
                                const YAML::Node& configNode, std::string* error) = 0;

        virtual bool publish(const model::PrimitiveFrame& frame, std::string* error) = 0;

        virtual bool ok() const = 0;

        virtual void shutdown() = 0;
    };

    std::unique_ptr<IDeviceOutputChannel> createDeviceOutputChannel(const std::string& channelType, std::string* error);

}  // namespace puppet::transport
