#include "puppet/transport/udp/udp_runtime_channel.hpp"

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

    UdpRuntimeChannel::UdpRuntimeChannel(const UdpRuntimeConfig& config) : config_(config) {}

    UdpRuntimeChannel::~UdpRuntimeChannel() {
        closeSocket(frameInputFd_);
        closeSocket(robotInputFd_);
        closeSocket(controlOutputFd_);
        started_ = false;
    }

    bool UdpRuntimeChannel::start(std::string& error) {
        return start(config_, error);
    }

    bool UdpRuntimeChannel::start(const UdpRuntimeConfig& config, std::string& error) {
        config_ = config;
        LOG(INFO) << "UdpRuntimeChannel start input_bind=" << config_.inputBindHost << ":" << config_.inputBindPort
                  << " output_remote=" << config_.outputRemoteHost << ":" << config_.outputControlIntentPort;

        if (!initDefaultEndpoints(error)) {
            setLastError(error);
            LOG(ERROR) << "UdpRuntimeChannel initDefaultEndpoints failed: " << error;
            return false;
        }

        started_ = true;
        error.clear();
        return true;
    }

    bool UdpRuntimeChannel::isRunning() const {
        return started_;
    }

    bool UdpRuntimeChannel::tryPopFrame(model::PrimitiveFrame& frame) {
        pollInputs();

        std::lock_guard<std::mutex> lock(queueMutex_);
        if (frameQueue_.empty()) {
            return false;
        }
        frame = std::move(*frameQueue_.front());
        frameQueue_.pop_front();
        return true;
    }

    bool UdpRuntimeChannel::publishControlIntent(const model::ControlIntent& intent, std::string& error) {
        if (controlOutputFd_ < 0) {
            error = "control intent udp socket is invalid";
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

        const ssize_t rc = sendto(controlOutputFd_, payload.data(), payload.size(), 0,
                                  reinterpret_cast<const sockaddr*>(&controlRemoteAddr_), sizeof(controlRemoteAddr_));
        if (rc < 0) {
            error = std::string("udp sendto failed: ") + std::strerror(errno);
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

    void UdpRuntimeChannel::registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        primitiveFrameHandlers_.push_back(std::move(handler));
    }

    void UdpRuntimeChannel::registerRobotStateFrameHandler(RobotStateFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        robotStateFrameHandlers_.push_back(std::move(handler));
    }

    UdpRuntimeChannel::RuntimeStats UdpRuntimeChannel::getRuntimeStats() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return stats_;
    }

    void UdpRuntimeChannel::setConfig(const UdpRuntimeConfig& config) {
        config_ = config;
    }

    bool UdpRuntimeChannel::initDefaultEndpoints(std::string& error) {
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

    bool UdpRuntimeChannel::initBuiltInInputEndpoint(const UdpRuntimeConfig::InputEndpointConfig& endpoint, std::string& error) {
        if (endpoint.key == "primitive_frame") {
            config_.inputBindHost = endpoint.bindHost;
            config_.inputBindPort = endpoint.bindPort;
            return createReceiverSocket(endpoint.bindHost, endpoint.bindPort, frameInputFd_, error);
        }
        if (endpoint.key == "robot_state_frame") {
            config_.robotStateInputBindHost = endpoint.bindHost;
            config_.robotStateInputBindPort = endpoint.bindPort;
            return createReceiverSocket(endpoint.bindHost, endpoint.bindPort, robotInputFd_, error);
        }
        return false;
    }

    bool UdpRuntimeChannel::initBuiltInOutputEndpoint(const UdpRuntimeConfig::OutputEndpointConfig& endpoint, std::string& error) {
        if (endpoint.key != "control_intent") {
            return false;
        }
        config_.outputRemoteHost        = endpoint.remoteHost;
        config_.outputControlIntentPort = endpoint.remotePort;
        return createSenderSocket(endpoint.remoteHost, endpoint.remotePort, controlOutputFd_, controlRemoteAddr_, error);
    }

    bool UdpRuntimeChannel::createReceiverSocket(const std::string& bindHost, uint16_t bindPort, int& fd, std::string& error) {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            error = std::string("create udp receiver socket failed: ") + std::strerror(errno);
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(bindPort);
        if (inet_pton(AF_INET, bindHost.c_str(), &addr.sin_addr) != 1) {
            error = "invalid bind ipv4 address: " + bindHost;
            close(sockfd);
            return false;
        }

        const int enableReuse = 1;
        setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enableReuse, sizeof(enableReuse));

        if (bind(sockfd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
            error = std::string("udp bind failed: ") + std::strerror(errno);
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

    bool UdpRuntimeChannel::createSenderSocket(const std::string& remoteHost, uint16_t remotePort, int& fd, sockaddr_in& remoteAddr,
                                               std::string& error) {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            error = std::string("create udp sender socket failed: ") + std::strerror(errno);
            return false;
        }

        remoteAddr            = {};
        remoteAddr.sin_family = AF_INET;
        remoteAddr.sin_port   = htons(remotePort);
        if (inet_pton(AF_INET, remoteHost.c_str(), &remoteAddr.sin_addr) != 1) {
            error = "invalid remote ipv4 address: " + remoteHost;
            close(sockfd);
            return false;
        }

        fd = sockfd;
        return true;
    }

    bool UdpRuntimeChannel::setNonBlocking(int fd, std::string& error) const {
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

    bool UdpRuntimeChannel::receiveAndDispatchInput(int fd, const std::string& endpointKey, bool pushToQueue, std::string& error) {
        std::vector<uint8_t> recvBuf(static_cast<size_t>(std::max(512, config_.recvBufferSize)));
        while (true) {
            sockaddr_in peer{};
            socklen_t peerLen = sizeof(peer);
            const ssize_t rc  = recvfrom(fd, recvBuf.data(), recvBuf.size(), 0, reinterpret_cast<sockaddr*>(&peer), &peerLen);
            if (rc < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    error.clear();
                    return true;
                }
                error = std::string("udp recvfrom failed: ") + std::strerror(errno);
                return false;
            }
            if (rc == 0) {
                continue;
            }

            handleIncomingPayload(endpointKey, recvBuf.data(), static_cast<size_t>(rc), pushToQueue);
        }
    }

    bool UdpRuntimeChannel::handleIncomingPayload(const std::string& endpointKey, const uint8_t* data, size_t size, bool pushToQueue) {
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

    void UdpRuntimeChannel::pollInputs() {
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
            if (!receiveAndDispatchInput(frameInputFd_, "primitive_frame", true, error)) {
                setLastError(error);
                LOG(ERROR) << "UdpRuntimeChannel read primitive_frame failed: " << error;
            }
        }
        if (robotInputFd_ >= 0 && FD_ISSET(robotInputFd_, &readSet)) {
            if (!receiveAndDispatchInput(robotInputFd_, "robot_state_frame", false, error)) {
                setLastError(error);
                LOG(ERROR) << "UdpRuntimeChannel read robot_state_frame failed: " << error;
            }
        }
    }

    void UdpRuntimeChannel::closeSocket(int& fd) noexcept {
        if (fd >= 0) {
            close(fd);
            fd = -1;
        }
    }

    void UdpRuntimeChannel::setLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.lastError = error;
    }

    ::puppet::puppet_proto::ControlIntent UdpRuntimeChannel::toProto(const model::ControlIntent& src) const {
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
