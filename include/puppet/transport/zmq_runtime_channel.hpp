#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "puppet/control_intent.pb.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/runtime_channel.hpp"
#include "puppet/transport/zmq_runtime_config.hpp"

namespace puppet::runtime {

    class ZmqRuntimeChannel : public IRuntimeChannel {
       public:
        using PrimitiveFrameHandler  = IRuntimeChannel::PrimitiveFrameHandler;
        using RobotStateFrameHandler = IRuntimeChannel::RobotStateFrameHandler;
        using RuntimeStats           = IRuntimeChannel::RuntimeStats;

        ZmqRuntimeChannel() = default;
        explicit ZmqRuntimeChannel(const ZmqRuntimeConfig& config);
        ~ZmqRuntimeChannel() override;

        bool start(std::string& error) override;
        bool start(const ZmqRuntimeConfig& config, std::string& error);
        bool isRunning() const override;
        bool tryPopFrame(model::PrimitiveFrame& frame) override;
        bool publishControlIntent(const model::ControlIntent& intent, std::string& error) override;
        void registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) override;
        void registerRobotStateFrameHandler(RobotStateFrameHandler handler) override;
        RuntimeStats getRuntimeStats() const override;
        int idleSleepMs() const override { return config_.idleSleepMs; }
        void setConfig(const ZmqRuntimeConfig& config);

       private:
        bool initContext(std::string& error);
        bool initDefaultEndpoints(std::string& error);
        bool initBuiltInInputEndpoint(const ZmqRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        bool initBuiltInOutputEndpoint(const ZmqRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        bool createSubscriber(const std::string& endpoint, const std::string& topic, void*& socket, std::string& error);
        bool createPublisher(const std::string& endpoint, void*& socket, std::string& error);
        bool tryReceiveMessage(void* socket, std::string& topic, std::string& payload, bool& gotMessage, std::string& error) const;
        bool onInputMessage(void* socket, const std::string& endpointKey, bool pushToQueue);
        void pollInputs();
        void closeSocket(void*& socket) noexcept;
        void destroyContext() noexcept;
        void setLastError(const std::string& error);

        ::puppet::puppet_proto::ControlIntent toProto(const model::ControlIntent& src) const;

        ZmqRuntimeConfig config_;
        bool started_ = false;

        mutable std::mutex queueMutex_;
        std::deque<std::shared_ptr<model::PrimitiveFrame>> frameQueue_;
        size_t maxQueueSize_ = 200;

        mutable std::mutex handlerMutex_;
        std::vector<PrimitiveFrameHandler> primitiveFrameHandlers_;
        std::vector<RobotStateFrameHandler> robotStateFrameHandlers_;

        mutable std::mutex statsMutex_;
        RuntimeStats stats_;
        std::map<std::string, size_t> readerEndpointCounts_;
        std::map<std::string, size_t> writerEndpointCounts_;

        void* zmqContext_       = nullptr;
        void* frameSubscriber_  = nullptr;
        void* robotSubscriber_  = nullptr;
        void* controlPublisher_ = nullptr;
    };

}  // namespace puppet::runtime
