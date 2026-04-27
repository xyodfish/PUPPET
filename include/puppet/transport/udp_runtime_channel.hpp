#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <netinet/in.h>

#include "puppet/control_intent.pb.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/runtime_channel.hpp"
#include "puppet/transport/udp_runtime_config.hpp"

namespace puppet::runtime {

    class UdpRuntimeChannel : public IRuntimeChannel {
       public:
        using PrimitiveFrameHandler  = IRuntimeChannel::PrimitiveFrameHandler;
        using RobotStateFrameHandler = IRuntimeChannel::RobotStateFrameHandler;
        using RuntimeStats           = IRuntimeChannel::RuntimeStats;

        UdpRuntimeChannel() = default;
        explicit UdpRuntimeChannel(const UdpRuntimeConfig& config);
        ~UdpRuntimeChannel() override;

        bool start(std::string& error) override;
        bool start(const UdpRuntimeConfig& config, std::string& error);
        bool isRunning() const override;
        bool tryPopFrame(model::PrimitiveFrame& frame) override;
        bool publishControlIntent(const model::ControlIntent& intent, std::string& error) override;
        void registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) override;
        void registerRobotStateFrameHandler(RobotStateFrameHandler handler) override;
        RuntimeStats getRuntimeStats() const override;
        int idleSleepMs() const override { return config_.idleSleepMs; }
        void setConfig(const UdpRuntimeConfig& config);

       private:
        bool initDefaultEndpoints(std::string& error);
        bool initBuiltInInputEndpoint(const UdpRuntimeConfig::InputEndpointConfig& endpoint, std::string& error);
        bool initBuiltInOutputEndpoint(const UdpRuntimeConfig::OutputEndpointConfig& endpoint, std::string& error);
        bool createReceiverSocket(const std::string& bindHost, uint16_t bindPort, int& fd, std::string& error);
        bool createSenderSocket(const std::string& remoteHost, uint16_t remotePort, int& fd, sockaddr_in& remoteAddr, std::string& error);
        bool setNonBlocking(int fd, std::string& error) const;
        bool receiveAndDispatchInput(int fd, const std::string& endpointKey, bool pushToQueue, std::string& error);
        bool handleIncomingPayload(const std::string& endpointKey, const uint8_t* data, size_t size, bool pushToQueue);
        void pollInputs();
        void closeSocket(int& fd) noexcept;
        void setLastError(const std::string& error);
        ::puppet::puppet_proto::ControlIntent toProto(const model::ControlIntent& src) const;

        UdpRuntimeConfig config_;
        bool started_ = false;

        mutable std::mutex queueMutex_;
        std::deque<std::shared_ptr<model::PrimitiveFrame>> frameQueue_;
        size_t maxQueueSize_ = 200;

        mutable std::mutex handlerMutex_;
        std::vector<PrimitiveFrameHandler> primitiveFrameHandlers_;
        std::vector<RobotStateFrameHandler> robotStateFrameHandlers_;

        mutable std::mutex statsMutex_;
        RuntimeStats stats_;

        int frameInputFd_    = -1;
        int robotInputFd_    = -1;
        int controlOutputFd_ = -1;
        sockaddr_in controlRemoteAddr_{};
    };

}  // namespace puppet::runtime
