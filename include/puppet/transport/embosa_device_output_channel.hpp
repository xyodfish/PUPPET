#pragma once

#include <memory>
#include <string>

#include "embosa.hpp"

#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/device_output_channel.hpp"

namespace puppet::transport {

    class EmbosaDeviceOutputChannel : public IDeviceOutputChannel {
       public:
        bool initialize(const std::string& nodeName, const std::string& topicName, const std::string& outputEndpoint,
                        const YAML::Node& configNode, std::string* error) override;

        bool publish(const model::PrimitiveFrame& frame, std::string* error) override;

        bool ok() const override;

        void shutdown() override;

       private:
        std::shared_ptr<galbot::embosa::Node> node_;
        std::shared_ptr<galbot::embosa::SerializationWriter<::puppet::puppet_proto::PrimitiveFrame>> writer_;
    };

}  // namespace puppet::transport
