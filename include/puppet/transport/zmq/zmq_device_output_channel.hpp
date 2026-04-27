#pragma once

#include <string>

#include "puppet/transport/device_output_channel.hpp"

namespace puppet::transport {

    class ZmqDeviceOutputChannel : public IDeviceOutputChannel {
       public:
        bool initialize(const std::string& nodeName, const std::string& topicName, const std::string& outputEndpoint,
                        const YAML::Node& configNode, std::string* error) override;

        bool publish(const model::PrimitiveFrame& frame, std::string* error) override;

        bool ok() const override;

        void shutdown() override;

       private:
        std::string topicName_;
        bool running_    = false;
        void* context_   = nullptr;
        void* publisher_ = nullptr;
    };

}  // namespace puppet::transport
