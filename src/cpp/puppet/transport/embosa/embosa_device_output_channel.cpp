#include "puppet/transport/embosa/embosa_device_output_channel.hpp"

#include "puppet/transport/proto_copy.hpp"

namespace puppet::transport {

    bool EmbosaDeviceOutputChannel::initialize(const std::string& nodeName, const std::string& topicName, const std::string& outputEndpoint,
                                               const YAML::Node& configNode, std::string* error) {
        (void)outputEndpoint;
        (void)configNode;

        galbot::embosa::EmbosaInit();
        node_ = galbot::embosa::CreateNode(nodeName);
        if (node_ == nullptr) {
            *error = "create embosa node failed";
            return false;
        }

        writer_ = node_->CreateWriter<::puppet::puppet_proto::PrimitiveFrame>(topicName);
        if (writer_ == nullptr) {
            *error = "create embosa writer failed";
            return false;
        }
        return true;
    }

    bool EmbosaDeviceOutputChannel::publish(const model::PrimitiveFrame& frame, std::string* error) {
        (void)error;
        auto message = std::make_shared<::puppet::puppet_proto::PrimitiveFrame>();
        if (!copyToProto(frame, message.get())) {
            if (error != nullptr) {
                *error = "convert model primitive frame to proto failed";
            }
            return false;
        }
        writer_->Publish(message);
        return true;
    }

    bool EmbosaDeviceOutputChannel::ok() const {
        return galbot::embosa::OK();
    }

    void EmbosaDeviceOutputChannel::shutdown() {
        writer_.reset();
        node_.reset();
        galbot::embosa::WaitForShutdown();
        galbot::embosa::Clear();
    }

}  // namespace puppet::transport
