#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "puppet/control_intent.pb.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/runtime_channel.hpp"
#include "puppet/transport/tcp_runtime_config.hpp"

namespace puppet::runtime {

    class TcpRuntimeChannel : public IRuntimeChannel {
       public:
        using PrimitiveFrameHandler  = IRuntimeChannel::PrimitiveFrameHandler;
        using RobotStateFrameHandler = IRuntimeChannel::RobotStateFrameHandler;
        using RuntimeStats           = IRuntimeChannel::RuntimeStats;

        TcpRuntimeChannel() = default;
        explicit TcpRuntimeChannel(const TcpRuntimeConfig& config);
        ~TcpRuntimeChannel() override;

        bool start(std::string& error) override;
        bool start(const TcpRuntimeConfig& config, std::string& error);
        bool isRunning() const override;
        bool tryPopFrame(model::PrimitiveFrame& frame) override;
        bool publishControlIntent(const model::ControlIntent& intent, std::string& error) override;
        void registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) override;
        void registerRobotStateFrameHandler(RobotStateFrameHandler handler) override;
        RuntimeStats getRuntimeStats() const override;
        int idleSleepMs() const override { return config_.idleSleepMs; }
        void setConfig(const TcpRuntimeConfig& config);

       private:
        bool initDefaultEndpoints(std::string& error);
        bool initBuiltInInputEndpoint(const TcpRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        bool initBuiltInOutputEndpoint(const TcpRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        bool createClientSocket(const std::string& host, uint16_t port, int& fd, std::string& error);
        bool setNonBlocking(int fd, std::string& error) const;
        bool sendAll(int fd, const uint8_t* data, size_t size, std::string& error) const;
        bool handleIncomingPayload(const std::string& endpointKey, const uint8_t* data, size_t size, bool pushToQueue);
        bool readAndDispatchInput(int fd, const std::string& endpointKey, bool pushToQueue, std::string& error);
        bool tryExtractFramedMessage(std::vector<uint8_t>& buffer, std::vector<uint8_t>& payload) const;
        void pollInputs();
        void closeSocket(int& fd) noexcept;
        void setLastError(const std::string& error);
        ::puppet::puppet_proto::ControlIntent toProto(const model::ControlIntent& src) const;

        TcpRuntimeConfig config_;
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

        std::unordered_map<int, std::vector<uint8_t>> inputBuffers_;
    };

}  // namespace puppet::runtime
