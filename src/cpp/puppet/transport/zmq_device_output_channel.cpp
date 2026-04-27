#include "puppet/transport/zmq_device_output_channel.hpp"

#include <cerrno>

#include <zmq.h>

#include "puppet/transport/proto_copy.hpp"

namespace puppet::transport {

    bool ZmqDeviceOutputChannel::initialize(const std::string& nodeName, const std::string& topicName, const std::string& outputEndpoint,
                                            const YAML::Node& configNode, std::string* error) {
        (void)nodeName;
        (void)configNode;

        topicName_ = topicName;
        context_   = zmq_ctx_new();
        if (context_ == nullptr) {
            *error = std::string("create zmq context failed: ") + zmq_strerror(errno);
            return false;
        }

        publisher_ = zmq_socket(context_, ZMQ_PUB);
        if (publisher_ == nullptr) {
            *error = std::string("create zmq publisher failed: ") + zmq_strerror(errno);
            shutdown();
            return false;
        }

        if (zmq_bind(publisher_, outputEndpoint.c_str()) != 0) {
            *error = std::string("bind zmq publisher failed: ") + zmq_strerror(errno);
            shutdown();
            return false;
        }

        running_ = true;
        return true;
    }

    bool ZmqDeviceOutputChannel::publish(const model::PrimitiveFrame& frame, std::string* error) {
        ::puppet::puppet_proto::PrimitiveFrame framePb;
        if (!copyToProto(frame, &framePb)) {
            *error = "convert model primitive frame to proto failed";
            return false;
        }

        std::string payload(framePb.ByteSizeLong(), '\0');
        if (!framePb.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
            *error = "serialize primitive frame failed";
            return false;
        }

        if (zmq_send(publisher_, topicName_.data(), topicName_.size(), ZMQ_SNDMORE) < 0) {
            *error = std::string("zmq send topic failed: ") + zmq_strerror(errno);
            return false;
        }
        if (zmq_send(publisher_, payload.data(), payload.size(), 0) < 0) {
            *error = std::string("zmq send payload failed: ") + zmq_strerror(errno);
            return false;
        }
        return true;
    }

    bool ZmqDeviceOutputChannel::ok() const {
        return running_;
    }

    void ZmqDeviceOutputChannel::shutdown() {
        running_ = false;
        if (publisher_ != nullptr) {
            zmq_close(publisher_);
            publisher_ = nullptr;
        }
        if (context_ != nullptr) {
            zmq_ctx_term(context_);
            context_ = nullptr;
        }
    }

}  // namespace puppet::transport
