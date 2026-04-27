#include "puppet/transport/tcp/tcp_runtime_channel.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

#include <algorithm>
#include <functional>

#include <glog/logging.h>

#include "puppet/transport/proto_copy.hpp"

namespace puppet::runtime {

    namespace {
        constexpr int kSelectTimeoutUsec = 0;
    }

    TcpRuntimeChannel::TcpRuntimeChannel(const TcpRuntimeConfig& config) : config_(config) {}

    TcpRuntimeChannel::~TcpRuntimeChannel() {
        closeSocket(frameInputFd_);
        closeSocket(robotInputFd_);
        closeSocket(controlOutputFd_);
        started_ = false;
    }

    bool TcpRuntimeChannel::start(std::string& error) {
        return start(config_, error);
    }

    bool TcpRuntimeChannel::start(const TcpRuntimeConfig& config, std::string& error) {
        config_ = config;
        LOG(INFO) << "TcpRuntimeChannel start input=" << config_.inputHost << ":" << config_.inputPort
                  << " output=" << config_.outputControlHost << ":" << config_.outputControlIntentPort;

        if (!initDefaultEndpoints(error)) {
            setLastError(error);
            LOG(ERROR) << "TcpRuntimeChannel initDefaultEndpoints failed: " << error;
            return false;
        }

        started_ = true;
        error.clear();
        return true;
    }

    bool TcpRuntimeChannel::isRunning() const {
        return started_;
    }

    bool TcpRuntimeChannel::tryPopFrame(model::PrimitiveFrame& frame) {
        pollInputs();

        std::lock_guard<std::mutex> lock(queueMutex_);
        if (frameQueue_.empty()) {
            return false;
        }
        frame = std::move(*frameQueue_.front());
        frameQueue_.pop_front();
        return true;
    }

    bool TcpRuntimeChannel::publishControlIntent(const model::ControlIntent& intent, std::string& error) {
        if (controlOutputFd_ < 0) {
            error = "control intent tcp socket is invalid";
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

        if (config_.useLengthPrefix) {
            uint32_t msgLen = htonl(static_cast<uint32_t>(payload.size()));
            if (!sendAll(controlOutputFd_, reinterpret_cast<const uint8_t*>(&msgLen), sizeof(msgLen), error)) {
                setLastError(error);
                return false;
            }
        }

        if (!sendAll(controlOutputFd_, reinterpret_cast<const uint8_t*>(payload.data()), payload.size(), error)) {
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

    void TcpRuntimeChannel::registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        primitiveFrameHandlers_.push_back(std::move(handler));
    }

    void TcpRuntimeChannel::registerRobotStateFrameHandler(RobotStateFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        robotStateFrameHandlers_.push_back(std::move(handler));
    }

    TcpRuntimeChannel::RuntimeStats TcpRuntimeChannel::getRuntimeStats() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return stats_;
    }

    void TcpRuntimeChannel::setConfig(const TcpRuntimeConfig& config) {
        config_ = config;
    }

    bool TcpRuntimeChannel::initDefaultEndpoints(std::string& error) {
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
        }
        return true;
    }

    bool TcpRuntimeChannel::initBuiltInInputEndpoint(const TcpRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key == "primitive_frame") {
            config_.inputHost = endpoint.host;
            config_.inputPort = endpoint.port;
            return createClientSocket(endpoint.host, endpoint.port, frameInputFd_, error);
        }
        if (endpoint.key == "robot_state_frame") {
            config_.robotStateInputHost = endpoint.host;
            config_.robotStateInputPort = endpoint.port;
            return createClientSocket(endpoint.host, endpoint.port, robotInputFd_, error);
        }
        return false;
    }

    bool TcpRuntimeChannel::initBuiltInOutputEndpoint(const TcpRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key != "control_intent") {
            return false;
        }
        config_.outputControlHost       = endpoint.host;
        config_.outputControlIntentPort = endpoint.port;
        return createClientSocket(endpoint.host, endpoint.port, controlOutputFd_, error);
    }

    bool TcpRuntimeChannel::createClientSocket(const std::string& host, uint16_t port, int& fd, std::string& error) {
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) {
            error = std::string("create tcp socket failed: ") + std::strerror(errno);
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port);
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            error = "invalid ipv4 address: " + host;
            close(sockfd);
            return false;
        }

        if (connect(sockfd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
            error = std::string("tcp connect failed: ") + std::strerror(errno);
            close(sockfd);
            return false;
        }

        if (!setNonBlocking(sockfd, error)) {
            close(sockfd);
            return false;
        }

        fd = sockfd;
        return true;
    }

    bool TcpRuntimeChannel::setNonBlocking(int fd, std::string& error) const {
        const int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0) {
            error = std::string("fcntl(F_GETFL) failed: ") + std::strerror(errno);
            return false;
        }
        if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            error = std::string("fcntl(F_SETFL, O_NONBLOCK) failed: ") + std::strerror(errno);
            return false;
        }
        return true;
    }

    bool TcpRuntimeChannel::sendAll(int fd, const uint8_t* data, size_t size, std::string& error) const {
        size_t sent = 0;
        while (sent < size) {
            const ssize_t rc = send(fd, data + sent, size - sent, 0);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                error = std::string("tcp send failed: ") + std::strerror(errno);
                return false;
            }
            if (rc == 0) {
                error = "tcp send returned 0";
                return false;
            }
            sent += static_cast<size_t>(rc);
        }
        return true;
    }

    bool TcpRuntimeChannel::handleIncomingPayload(const std::string& endpointKey, const uint8_t* data, size_t size, bool pushToQueue) {
        ::puppet::puppet_proto::PrimitiveFrame framePb;
        if (!framePb.ParseFromArray(data, static_cast<int>(size))) {
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

    bool TcpRuntimeChannel::readAndDispatchInput(int fd, const std::string& endpointKey, bool pushToQueue, std::string& error) {
        std::vector<uint8_t> recvBuf(static_cast<size_t>(std::max(256, config_.recvBufferSize)));
        bool receivedAnyData = false;
        while (true) {
            const ssize_t rc = recv(fd, recvBuf.data(), recvBuf.size(), 0);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                error = std::string("tcp recv failed: ") + std::strerror(errno);
                return false;
            }
            if (rc == 0) {
                error = "tcp peer closed connection";
                return false;
            }

            receivedAnyData = true;
            if (!config_.useLengthPrefix) {
                handleIncomingPayload(endpointKey, recvBuf.data(), static_cast<size_t>(rc), pushToQueue);
                continue;
            }

            auto& staging = inputBuffers_[fd];
            staging.insert(staging.end(), recvBuf.begin(), recvBuf.begin() + rc);
            std::vector<uint8_t> payload;
            while (tryExtractFramedMessage(staging, payload)) {
                handleIncomingPayload(endpointKey, payload.data(), payload.size(), pushToQueue);
                payload.clear();
            }
        }

        (void)receivedAnyData;
        error.clear();
        return true;
    }

    bool TcpRuntimeChannel::tryExtractFramedMessage(std::vector<uint8_t>& buffer, std::vector<uint8_t>& payload) const {
        if (buffer.size() < sizeof(uint32_t)) {
            return false;
        }

        uint32_t netLength = 0;
        std::memcpy(&netLength, buffer.data(), sizeof(uint32_t));
        const uint32_t msgLen = ntohl(netLength);
        if (msgLen == 0 || msgLen > 32U * 1024U * 1024U) {
            return false;
        }
        if (buffer.size() < sizeof(uint32_t) + msgLen) {
            return false;
        }

        payload.assign(buffer.begin() + sizeof(uint32_t), buffer.begin() + sizeof(uint32_t) + msgLen);
        buffer.erase(buffer.begin(), buffer.begin() + sizeof(uint32_t) + msgLen);
        return true;
    }

    void TcpRuntimeChannel::pollInputs() {
        fd_set readSet;
        FD_ZERO(&readSet);

        int maxFd = -1;
        if (frameInputFd_ >= 0) {
            FD_SET(frameInputFd_, &readSet);
            maxFd = std::max(maxFd, frameInputFd_);
        }
        if (robotInputFd_ >= 0) {
            FD_SET(robotInputFd_, &readSet);
            maxFd = std::max(maxFd, robotInputFd_);
        }
        if (maxFd < 0) {
            return;
        }

        timeval timeout{};
        timeout.tv_sec  = 0;
        timeout.tv_usec = kSelectTimeoutUsec;
        const int ready = select(maxFd + 1, &readSet, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            return;
        }

        std::string error;
        if (frameInputFd_ >= 0 && FD_ISSET(frameInputFd_, &readSet)) {
            if (!readAndDispatchInput(frameInputFd_, "primitive_frame", true, error)) {
                setLastError(error);
                LOG(ERROR) << "TcpRuntimeChannel read primitive_frame failed: " << error;
            }
        }
        if (robotInputFd_ >= 0 && FD_ISSET(robotInputFd_, &readSet)) {
            if (!readAndDispatchInput(robotInputFd_, "robot_state_frame", false, error)) {
                setLastError(error);
                LOG(ERROR) << "TcpRuntimeChannel read robot_state_frame failed: " << error;
            }
        }
    }

    void TcpRuntimeChannel::closeSocket(int& fd) noexcept {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

    void TcpRuntimeChannel::setLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.lastError = error;
    }

    ::puppet::puppet_proto::ControlIntent TcpRuntimeChannel::toProto(const model::ControlIntent& src) const {
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
