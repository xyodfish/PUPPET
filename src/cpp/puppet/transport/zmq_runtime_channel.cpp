#include "puppet/transport/zmq_runtime_channel.hpp"

#include <cerrno>
#include <cstring>
#include <functional>
#include <utility>

#include <glog/logging.h>
#include <zmq.h>

#include "puppet/transport/proto_copy.hpp"

namespace puppet::runtime {

    ZmqRuntimeChannel::ZmqRuntimeChannel(const ZmqRuntimeConfig& config) : config_(config) {}

    ZmqRuntimeChannel::~ZmqRuntimeChannel() {
        closeSocket(frameSubscriber_);
        closeSocket(robotSubscriber_);
        closeSocket(controlPublisher_);
        destroyContext();
        started_ = false;
    }

    bool ZmqRuntimeChannel::start(std::string& error) {
        return start(config_, error);
    }

    bool ZmqRuntimeChannel::start(const ZmqRuntimeConfig& config, std::string& error) {
        config_ = config;
        LOG(INFO) << "ZmqRuntimeChannel start input_endpoint=" << config_.inputEndpoint
                  << " output_endpoint=" << config_.outputControlIntentEndpoint;

        if (!initContext(error)) {
            setLastError(error);
            LOG(ERROR) << "ZmqRuntimeChannel initContext failed: " << error;
            return false;
        }

        if (!initDefaultEndpoints(error)) {
            setLastError(error);
            LOG(ERROR) << "ZmqRuntimeChannel initDefaultEndpoints failed: " << error;
            return false;
        }

        started_ = true;
        error.clear();
        return true;
    }

    bool ZmqRuntimeChannel::isRunning() const {
        return started_;
    }

    bool ZmqRuntimeChannel::tryPopFrame(model::PrimitiveFrame& frame) {
        pollInputs();

        std::lock_guard<std::mutex> lock(queueMutex_);
        if (frameQueue_.empty()) {
            return false;
        }
        frame = std::move(*frameQueue_.front());
        frameQueue_.pop_front();
        return true;
    }

    bool ZmqRuntimeChannel::publishControlIntent(const model::ControlIntent& intent, std::string& error) {
        if (controlPublisher_ == nullptr) {
            error = "control intent publisher is null";
            setLastError(error);
            return false;
        }

        const ::puppet::puppet_proto::ControlIntent controlIntentPb = toProto(intent);
        std::string payload(controlIntentPb.ByteSizeLong(), '\0');
        if (!controlIntentPb.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
            error = "serialize control intent proto failed";
            setLastError(error);
            return false;
        }

        if (!config_.outputControlIntentTopic.empty()) {
            const int topicRc =
                zmq_send(controlPublisher_, config_.outputControlIntentTopic.data(), config_.outputControlIntentTopic.size(), ZMQ_SNDMORE);
            if (topicRc < 0) {
                error = std::string("zmq publish topic failed: ") + zmq_strerror(errno);
                setLastError(error);
                return false;
            }
        }

        const int payloadRc = zmq_send(controlPublisher_, payload.data(), payload.size(), 0);
        if (payloadRc < 0) {
            error = std::string("zmq publish payload failed: ") + zmq_strerror(errno);
            setLastError(error);
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.publishedControlIntentCount++;
        }
        error.clear();
        return true;
    }

    void ZmqRuntimeChannel::registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        primitiveFrameHandlers_.push_back(std::move(handler));
    }

    void ZmqRuntimeChannel::registerRobotStateFrameHandler(RobotStateFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        robotStateFrameHandlers_.push_back(std::move(handler));
    }

    ZmqRuntimeChannel::RuntimeStats ZmqRuntimeChannel::getRuntimeStats() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return stats_;
    }

    void ZmqRuntimeChannel::setConfig(const ZmqRuntimeConfig& config) {
        config_ = config;
    }

    bool ZmqRuntimeChannel::initContext(std::string& error) {
        zmqContext_ = zmq_ctx_new();
        if (zmqContext_ == nullptr) {
            error = std::string("create zmq context failed: ") + zmq_strerror(errno);
            return false;
        }

        const int ioThreads = config_.ioThreads;
        if (zmq_ctx_set(zmqContext_, ZMQ_IO_THREADS, ioThreads) != 0) {
            error = std::string("set ZMQ_IO_THREADS failed: ") + zmq_strerror(errno);
            return false;
        }
        return true;
    }

    bool ZmqRuntimeChannel::initDefaultEndpoints(std::string& error) {
        for (const auto& endpoint : config_.inputEndpoints) {
            if (!endpoint.enabled) {
                continue;
            }
            if (!initBuiltInInputEndpoint(endpoint, error)) {
                if (!error.empty()) {
                    return false;
                }
                error = "input endpoint key not supported: " + endpoint.key;
                return false;
            }
            LOG(INFO) << "ZmqRuntimeChannel input endpoint key=" << endpoint.key << " endpoint=" << endpoint.endpoint
                      << " topic=" << endpoint.topicName;
        }

        for (const auto& endpoint : config_.outputEndpoints) {
            if (!endpoint.enabled) {
                continue;
            }
            if (!initBuiltInOutputEndpoint(endpoint, error)) {
                if (!error.empty()) {
                    return false;
                }
                error = "output endpoint key not supported: " + endpoint.key;
                return false;
            }
            LOG(INFO) << "ZmqRuntimeChannel output endpoint key=" << endpoint.key << " endpoint=" << endpoint.endpoint
                      << " topic=" << endpoint.topicName;
        }
        return true;
    }

    bool ZmqRuntimeChannel::initBuiltInInputEndpoint(const ZmqRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key == "primitive_frame") {
            config_.inputEndpoint  = endpoint.endpoint;
            config_.inputTopicName = endpoint.topicName;
            if (!createSubscriber(endpoint.endpoint, endpoint.topicName, frameSubscriber_, error)) {
                return false;
            }
            readerEndpointCounts_[endpoint.endpoint]++;
            return true;
        }
        if (endpoint.key == "robot_state_frame") {
            config_.robotStateInputEndpoint  = endpoint.endpoint;
            config_.robotStateInputTopicName = endpoint.topicName;
            if (!createSubscriber(endpoint.endpoint, endpoint.topicName, robotSubscriber_, error)) {
                return false;
            }
            readerEndpointCounts_[endpoint.endpoint]++;
            return true;
        }
        return false;
    }

    bool ZmqRuntimeChannel::initBuiltInOutputEndpoint(const ZmqRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key != "control_intent") {
            return false;
        }
        config_.outputControlIntentEndpoint = endpoint.endpoint;
        config_.outputControlIntentTopic    = endpoint.topicName;
        if (!createPublisher(endpoint.endpoint, controlPublisher_, error)) {
            return false;
        }
        writerEndpointCounts_[endpoint.endpoint]++;
        return true;
    }

    bool ZmqRuntimeChannel::createSubscriber(const std::string& endpoint, const std::string& topic, void*& socket, std::string& error) {
        if (zmqContext_ == nullptr) {
            error = "zmq context is null";
            return false;
        }

        void* createdSocket = zmq_socket(zmqContext_, ZMQ_SUB);
        if (createdSocket == nullptr) {
            error = std::string("create subscriber failed: ") + zmq_strerror(errno);
            return false;
        }

        const int recvHwm = config_.recvHighWaterMark;
        if (zmq_setsockopt(createdSocket, ZMQ_RCVHWM, &recvHwm, sizeof(recvHwm)) != 0) {
            error = std::string("set ZMQ_RCVHWM failed: ") + zmq_strerror(errno);
            zmq_close(createdSocket);
            return false;
        }

        if (zmq_setsockopt(createdSocket, ZMQ_SUBSCRIBE, topic.data(), topic.size()) != 0) {
            error = std::string("set ZMQ_SUBSCRIBE failed: ") + zmq_strerror(errno);
            zmq_close(createdSocket);
            return false;
        }

        if (zmq_connect(createdSocket, endpoint.c_str()) != 0) {
            error = std::string("connect subscriber failed: ") + zmq_strerror(errno);
            zmq_close(createdSocket);
            return false;
        }

        socket = createdSocket;
        return true;
    }

    bool ZmqRuntimeChannel::createPublisher(const std::string& endpoint, void*& socket, std::string& error) {
        if (zmqContext_ == nullptr) {
            error = "zmq context is null";
            return false;
        }

        void* createdSocket = zmq_socket(zmqContext_, ZMQ_PUB);
        if (createdSocket == nullptr) {
            error = std::string("create publisher failed: ") + zmq_strerror(errno);
            return false;
        }

        const int sendHwm = config_.sendHighWaterMark;
        if (zmq_setsockopt(createdSocket, ZMQ_SNDHWM, &sendHwm, sizeof(sendHwm)) != 0) {
            error = std::string("set ZMQ_SNDHWM failed: ") + zmq_strerror(errno);
            zmq_close(createdSocket);
            return false;
        }

        if (zmq_bind(createdSocket, endpoint.c_str()) != 0) {
            error = std::string("bind publisher failed: ") + zmq_strerror(errno);
            zmq_close(createdSocket);
            return false;
        }

        socket = createdSocket;
        return true;
    }

    bool ZmqRuntimeChannel::tryReceiveMessage(void* socket, std::string& topic, std::string& payload, bool& gotMessage,
                                              std::string& error) const {
        gotMessage = false;
        topic.clear();
        payload.clear();

        zmq_msg_t firstPart;
        if (zmq_msg_init(&firstPart) != 0) {
            error = std::string("zmq_msg_init failed: ") + zmq_strerror(errno);
            return false;
        }
        const int firstRc = zmq_msg_recv(&firstPart, socket, ZMQ_DONTWAIT);
        if (firstRc < 0) {
            zmq_msg_close(&firstPart);
            if (errno == EAGAIN) {
                error.clear();
                return true;
            }
            error = std::string("recv message failed: ") + zmq_strerror(errno);
            return false;
        }
        gotMessage = true;

        int more      = 0;
        size_t moreSz = sizeof(more);
        if (zmq_getsockopt(socket, ZMQ_RCVMORE, &more, &moreSz) != 0) {
            zmq_msg_close(&firstPart);
            error = std::string("getsockopt ZMQ_RCVMORE failed: ") + zmq_strerror(errno);
            return false;
        }

        if (more != 0) {
            topic.assign(static_cast<const char*>(zmq_msg_data(&firstPart)), zmq_msg_size(&firstPart));
            zmq_msg_close(&firstPart);

            zmq_msg_t secondPart;
            if (zmq_msg_init(&secondPart) != 0) {
                error = std::string("zmq_msg_init second part failed: ") + zmq_strerror(errno);
                return false;
            }
            const int secondRc = zmq_msg_recv(&secondPart, socket, 0);
            if (secondRc < 0) {
                zmq_msg_close(&secondPart);
                error = std::string("recv payload failed: ") + zmq_strerror(errno);
                return false;
            }
            payload.assign(static_cast<const char*>(zmq_msg_data(&secondPart)), zmq_msg_size(&secondPart));
            zmq_msg_close(&secondPart);
        } else {
            payload.assign(static_cast<const char*>(zmq_msg_data(&firstPart)), zmq_msg_size(&firstPart));
            zmq_msg_close(&firstPart);
        }

        error.clear();
        return true;
    }

    bool ZmqRuntimeChannel::onInputMessage(void* socket, const std::string& endpointKey, bool pushToQueue) {
        std::string topic;
        std::string payload;
        bool gotMessage = false;
        std::string error;
        if (!tryReceiveMessage(socket, topic, payload, gotMessage, error)) {
            setLastError(error);
            LOG(ERROR) << "ZmqRuntimeChannel receive failed key=" << endpointKey << " error=" << error;
            return false;
        }
        if (!gotMessage) {
            return false;
        }

        ::puppet::puppet_proto::PrimitiveFrame framePb;
        if (!framePb.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.droppedPrimitiveFrameCount++;
            stats_.lastError = "parse primitive frame proto failed";
            return false;
        }

        model::PrimitiveFrame frame;
        if (!puppet::transport::copyFromProto(framePb, &frame)) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.droppedPrimitiveFrameCount++;
            stats_.lastError = "copy primitive frame proto failed";
            return false;
        }

        if (endpointKey == "primitive_frame") {
            {
                std::lock_guard<std::mutex> lock(statsMutex_);
                stats_.receivedPrimitiveFrameCount++;
            }
            {
                std::lock_guard<std::mutex> lock(handlerMutex_);
                for (const auto& handler : primitiveFrameHandlers_) {
                    if (handler) {
                        handler(frame);
                    }
                }
            }
            if (pushToQueue) {
                std::lock_guard<std::mutex> lock(queueMutex_);
                frameQueue_.push_back(std::make_shared<model::PrimitiveFrame>(std::move(frame)));
                while (frameQueue_.size() > maxQueueSize_) {
                    frameQueue_.pop_front();
                }
            }
            return true;
        }

        if (endpointKey == "robot_state_frame") {
            {
                std::lock_guard<std::mutex> lock(statsMutex_);
                stats_.receivedRobotStateFrameCount++;
            }
            std::lock_guard<std::mutex> lock(handlerMutex_);
            for (const auto& handler : robotStateFrameHandlers_) {
                if (handler) {
                    handler(frame);
                }
            }
            return true;
        }

        return false;
    }

    void ZmqRuntimeChannel::pollInputs() {
        std::vector<zmq_pollitem_t> pollItems;
        std::vector<std::pair<void*, std::string>> sockets;
        pollItems.reserve(2);
        sockets.reserve(2);

        if (frameSubscriber_ != nullptr) {
            pollItems.push_back(zmq_pollitem_t{frameSubscriber_, 0, ZMQ_POLLIN, 0});
            sockets.emplace_back(frameSubscriber_, "primitive_frame");
        }
        if (robotSubscriber_ != nullptr) {
            pollItems.push_back(zmq_pollitem_t{robotSubscriber_, 0, ZMQ_POLLIN, 0});
            sockets.emplace_back(robotSubscriber_, "robot_state_frame");
        }
        if (pollItems.empty()) {
            return;
        }

        const int ready = zmq_poll(pollItems.data(), pollItems.size(), 0);
        if (ready <= 0) {
            return;
        }

        for (size_t i = 0; i < pollItems.size(); ++i) {
            if ((pollItems[i].revents & ZMQ_POLLIN) == 0) {
                continue;
            }
            while (onInputMessage(sockets[i].first, sockets[i].second, sockets[i].second == "primitive_frame")) {}
        }
    }

    void ZmqRuntimeChannel::closeSocket(void*& socket) noexcept {
        if (socket != nullptr) {
            zmq_close(socket);
            socket = nullptr;
        }
    }

    void ZmqRuntimeChannel::destroyContext() noexcept {
        if (zmqContext_ != nullptr) {
            zmq_ctx_term(zmqContext_);
            zmqContext_ = nullptr;
        }
    }

    void ZmqRuntimeChannel::setLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.lastError = error;
    }

    ::puppet::puppet_proto::ControlIntent ZmqRuntimeChannel::toProto(const model::ControlIntent& src) const {
        ::puppet::puppet_proto::ControlIntent dst;
        dst.set_sequence_id(src.sequenceId);
        dst.mutable_context()->set_source_id(src.context.sourceId);
        dst.mutable_context()->set_mode(src.context.mode);
        dst.mutable_context()->set_semantic_context(src.context.semanticContext);
        dst.mutable_context()->set_pipeline_id(src.context.pipelineId);

        for (const auto& group : src.groupIntents) {
            auto* groupPb = dst.add_group_intents();
            groupPb->set_owner_source_id(group.ownerSourceId);
            groupPb->set_mode(group.mode);
            groupPb->set_priority(group.priority);
            groupPb->set_backend_hint(group.backendHint);
            groupPb->set_enabled(group.enabled);

            for (const auto& jointIntent : group.jointCommandIntents) {
                auto* jointPb = groupPb->add_joint_command_intents();
                for (const auto& n : jointIntent.jointNames) {
                    jointPb->add_joint_names(n);
                }
                for (double v : jointIntent.position) {
                    jointPb->add_position(v);
                }
                for (double v : jointIntent.velocity) {
                    jointPb->add_velocity(v);
                }
                for (double v : jointIntent.effort) {
                    jointPb->add_effort(v);
                }
                for (double v : jointIntent.stiffness) {
                    jointPb->add_stiffness(v);
                }
                for (double v : jointIntent.damping) {
                    jointPb->add_damping(v);
                }
                jointPb->set_weight(jointIntent.weight);
            }
        }
        return dst;
    }

}  // namespace puppet::runtime
