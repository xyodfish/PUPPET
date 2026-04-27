#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "embosa.hpp"
#include "puppet/control/control_intent_types.hpp"
#include "puppet/control_intent.pb.h"
#include "puppet/primitive/primitive_types.hpp"
#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/embosa/embosa_runtime_config.hpp"
#include "puppet/transport/runtime_channel.hpp"

namespace puppet::runtime {

    class EmbosaRuntimeChannel : public IRuntimeChannel {
       public:
        using PrimitiveFrameHandler  = IRuntimeChannel::PrimitiveFrameHandler;
        using RobotStateFrameHandler = IRuntimeChannel::RobotStateFrameHandler;
        using EndpointBinder         = std::function<bool(const std::string& topicName, std::string& error)>;
        using RuntimeStats           = IRuntimeChannel::RuntimeStats;

        EmbosaRuntimeChannel() = default;
        explicit EmbosaRuntimeChannel(const EmbosaRuntimeConfig& config);
        ~EmbosaRuntimeChannel() override;

        bool start(std::string& error) override;
        bool start(const EmbosaRuntimeConfig& config, std::string& error);
        bool isRunning() const override;

        bool tryPopFrame(model::PrimitiveFrame& frame) override;
        bool publishControlIntent(const model::ControlIntent& intent, std::string& error) override;
        void registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) override;
        void registerRobotStateFrameHandler(RobotStateFrameHandler handler) override;
        void registerInputEndpointBinder(const std::string& key, EndpointBinder binder);
        void registerOutputEndpointBinder(const std::string& key, EndpointBinder binder);
        RuntimeStats getRuntimeStats() const override;
        int idleSleepMs() const override { return config_.idleSleepMs; }
        void setConfig(const EmbosaRuntimeConfig& config);

        template <typename ProtoT>
        bool createReader(const std::string& topicName,
                          const std::function<void(const std::shared_ptr<ProtoT>& msg, const void* userData)>& callback,
                          std::shared_ptr<galbot::embosa::SerializationReader<ProtoT>>& reader, std::string& error) {
            if (node_ == nullptr) {
                error = "embosa node is null";
                return false;
            }
            auto createdReader = node_->CreateReader<ProtoT>(topicName, callback);
            if (createdReader == nullptr) {
                error = "create reader failed, topic=" + topicName;
                return false;
            }
            reader = createdReader;
            readerKeepAlive_.push_back(createdReader);
            readerTopicCounts_[topicName]++;
            error.clear();
            return true;
        }

        template <typename ProtoT>
        bool createWriter(const std::string& topicName, std::shared_ptr<galbot::embosa::SerializationWriter<ProtoT>>& writer,
                          std::string& error) {
            if (node_ == nullptr) {
                error = "embosa node is null";
                return false;
            }
            auto createdWriter = node_->CreateWriter<ProtoT>(topicName);
            if (createdWriter == nullptr) {
                error = "create writer failed, topic=" + topicName;
                return false;
            }
            writer = createdWriter;
            writerKeepAlive_.push_back(createdWriter);
            writerTopicCounts_[topicName]++;
            error.clear();
            return true;
        }

       private:
        bool initNode(std::string& error);
        bool initDefaultEndpoints(std::string& error);
        bool initBuiltInInputEndpoint(const EmbosaRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        bool initBuiltInOutputEndpoint(const EmbosaRuntimeConfig::EndpointConfig& endpoint, std::string& error);
        void setLastError(const std::string& error);

        void onPrimitiveFrame(const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg);
        void onRobotStateFrame(const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg);
        ::puppet::puppet_proto::ControlIntent toProto(const model::ControlIntent& src) const;

        EmbosaRuntimeConfig config_;
        bool started_ = false;

        std::mutex queueMutex_;
        std::deque<std::shared_ptr<model::PrimitiveFrame>> frameQueue_;
        size_t maxQueueSize_ = 200;

        mutable std::mutex handlerMutex_;
        std::vector<PrimitiveFrameHandler> primitiveFrameHandlers_;
        std::vector<RobotStateFrameHandler> robotStateFrameHandlers_;

        mutable std::mutex statsMutex_;
        RuntimeStats stats_;
        std::map<std::string, size_t> readerTopicCounts_;
        std::map<std::string, size_t> writerTopicCounts_;
        std::unordered_map<std::string, EndpointBinder> inputEndpointBinders_;
        std::unordered_map<std::string, EndpointBinder> outputEndpointBinders_;

        std::unique_ptr<galbot::embosa::Node> node_;
        std::shared_ptr<galbot::embosa::SerializationReader<::puppet::puppet_proto::PrimitiveFrame>> frameReader_;
        std::shared_ptr<galbot::embosa::SerializationReader<::puppet::puppet_proto::PrimitiveFrame>> robotStateReader_;
        std::shared_ptr<galbot::embosa::SerializationWriter<::puppet::puppet_proto::ControlIntent>> controlIntentWriter_;
        std::vector<std::shared_ptr<void>> readerKeepAlive_;
        std::vector<std::shared_ptr<void>> writerKeepAlive_;
    };

}  // namespace puppet::runtime
