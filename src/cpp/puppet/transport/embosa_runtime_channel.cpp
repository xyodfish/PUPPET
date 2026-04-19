#include "puppet/transport/embosa_runtime_channel.hpp"

#include <functional>

#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "puppet/transport/proto_copy.hpp"

namespace puppet::runtime {
    EmbosaRuntimeChannel::~EmbosaRuntimeChannel() {
        if (!started_) {
            return;
        }
        galbot::embosa::WaitForShutdown();
        galbot::embosa::Clear();
        started_ = false;
    }

    bool EmbosaRuntimeChannel::start(const EmbosaRuntimeConfig& config, std::string& error) {
        config_ = config;
        LOG(INFO) << "EmbosaRuntimeChannel start node=" << config_.embosaNodeName << " input_topic=" << config_.inputTopicName
                  << " output_topic=" << config_.outputControlIntentTopic;

        if (!initNode(error)) {
            setLastError(error);
            LOG(ERROR) << "EmbosaRuntimeChannel initNode failed: " << error;
            return false;
        }

        if (!initDefaultEndpoints(error)) {
            setLastError(error);
            LOG(ERROR) << "EmbosaRuntimeChannel initDefaultEndpoints failed: " << error;
            return false;
        }

        started_ = true;
        error.clear();
        return true;
    }

    bool EmbosaRuntimeChannel::isRunning() const {
        return galbot::embosa::OK();
    }

    bool EmbosaRuntimeChannel::tryPopFrame(model::PrimitiveFrame& frame) {
        std::lock_guard<std::mutex> lock(queueMutex_);
        if (frameQueue_.empty()) {
            return false;
        }
        frame = std::move(*frameQueue_.front());
        frameQueue_.pop_front();
        return true;
    }

    bool EmbosaRuntimeChannel::publishControlIntent(const model::ControlIntent& intent, std::string& error) {
        if (controlIntentWriter_ == nullptr) {
            error = "control intent writer is null";
            setLastError(error);
            LOG(ERROR) << "EmbosaRuntimeChannel publish failed: " << error;
            return false;
        }

        auto controlIntentPb = std::make_shared<::puppet::puppet_proto::ControlIntent>(toProto(intent));
        controlIntentWriter_->Publish(controlIntentPb);
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.publishedControlIntentCount++;
        }
        error.clear();
        return true;
    }

    void EmbosaRuntimeChannel::registerPrimitiveFrameHandler(PrimitiveFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        primitiveFrameHandlers_.push_back(std::move(handler));
    }

    void EmbosaRuntimeChannel::registerRobotStateFrameHandler(RobotStateFrameHandler handler) {
        std::lock_guard<std::mutex> lock(handlerMutex_);
        robotStateFrameHandlers_.push_back(std::move(handler));
    }

    void EmbosaRuntimeChannel::registerInputEndpointBinder(const std::string& key, EndpointBinder binder) {
        inputEndpointBinders_[key] = std::move(binder);
    }

    void EmbosaRuntimeChannel::registerOutputEndpointBinder(const std::string& key, EndpointBinder binder) {
        outputEndpointBinders_[key] = std::move(binder);
    }

    EmbosaRuntimeChannel::RuntimeStats EmbosaRuntimeChannel::getRuntimeStats() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return stats_;
    }

    bool EmbosaRuntimeChannel::initNode(std::string& error) {
        galbot::embosa::EmbosaInit();
        node_ = galbot::embosa::CreateNode(config_.embosaNodeName);
        if (node_ == nullptr) {
            error = "create embosa node failed";
            return false;
        }
        return true;
    }

    bool EmbosaRuntimeChannel::initDefaultEndpoints(std::string& error) {
        for (const auto& endpoint : config_.inputEndpoints) {
            if (!endpoint.enabled) {
                continue;
            }
            if (initBuiltInInputEndpoint(endpoint, error)) {
                LOG(INFO) << "EmbosaRuntimeChannel input endpoint key=" << endpoint.key << " topic=" << endpoint.topicName;
                continue;
            }
            const auto binderIt = inputEndpointBinders_.find(endpoint.key);
            if (binderIt == inputEndpointBinders_.end()) {
                error = "input endpoint key not supported: " + endpoint.key;
                return false;
            }
            if (!binderIt->second(endpoint.topicName, error)) {
                return false;
            }
            LOG(INFO) << "EmbosaRuntimeChannel input endpoint bound by external binder key=" << endpoint.key
                      << " topic=" << endpoint.topicName;
        }
        for (const auto& endpoint : config_.outputEndpoints) {
            if (!endpoint.enabled) {
                continue;
            }
            if (initBuiltInOutputEndpoint(endpoint, error)) {
                LOG(INFO) << "EmbosaRuntimeChannel output endpoint key=" << endpoint.key << " topic=" << endpoint.topicName;
                continue;
            }
            const auto binderIt = outputEndpointBinders_.find(endpoint.key);
            if (binderIt == outputEndpointBinders_.end()) {
                error = "output endpoint key not supported: " + endpoint.key;
                return false;
            }
            if (!binderIt->second(endpoint.topicName, error)) {
                return false;
            }
            LOG(INFO) << "EmbosaRuntimeChannel output endpoint bound by external binder key=" << endpoint.key
                      << " topic=" << endpoint.topicName;
        }
        return true;
    }

    bool EmbosaRuntimeChannel::initBuiltInInputEndpoint(const EmbosaRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key == "primitive_frame") {
            auto onMsg = [this](const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg, const void*) {
                onPrimitiveFrame(msg);
            };
            return createReader<::puppet::puppet_proto::PrimitiveFrame>(endpoint.topicName, onMsg, frameReader_, error);
        }
        if (endpoint.key == "robot_state_frame") {
            auto onMsg = [this](const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg, const void*) {
                onRobotStateFrame(msg);
            };
            return createReader<::puppet::puppet_proto::PrimitiveFrame>(endpoint.topicName, onMsg, robotStateReader_, error);
        }
        return false;
    }

    bool EmbosaRuntimeChannel::initBuiltInOutputEndpoint(const EmbosaRuntimeConfig::EndpointConfig& endpoint, std::string& error) {
        if (endpoint.key != "control_intent") {
            return false;
        }
        return createWriter<::puppet::puppet_proto::ControlIntent>(endpoint.topicName, controlIntentWriter_, error);
    }

    void EmbosaRuntimeChannel::setLastError(const std::string& error) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.lastError = error;
    }

    void EmbosaRuntimeChannel::onPrimitiveFrame(const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg) {
        if (msg == nullptr) {
            return;
        }

        auto frame = std::make_shared<model::PrimitiveFrame>();
        if (!puppet::transport::copyFromProto(*msg, frame.get())) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.droppedPrimitiveFrameCount++;
            stats_.lastError = "invalid primitive frame proto";
            LOG(WARNING) << "EmbosaRuntimeChannel drop invalid PrimitiveFrame proto";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.receivedPrimitiveFrameCount++;
        }
        {
            std::lock_guard<std::mutex> lock(handlerMutex_);
            for (const auto& handler : primitiveFrameHandlers_) {
                if (handler) {
                    handler(*frame);
                }
            }
        }

        std::lock_guard<std::mutex> lock(queueMutex_);
        frameQueue_.push_back(frame);
        while (frameQueue_.size() > maxQueueSize_) {
            frameQueue_.pop_front();
        }
    }

    void EmbosaRuntimeChannel::onRobotStateFrame(const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg) {
        if (msg == nullptr) {
            return;
        }

        model::PrimitiveFrame frame;
        if (!puppet::transport::copyFromProto(*msg, &frame)) {
            std::lock_guard<std::mutex> lock(statsMutex_);
            stats_.droppedPrimitiveFrameCount++;
            stats_.lastError = "invalid robot state primitive frame proto";
            LOG(WARNING) << "EmbosaRuntimeChannel drop invalid robot state PrimitiveFrame proto";
            return;
        }

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
    }

    ::puppet::puppet_proto::ControlIntent EmbosaRuntimeChannel::toProto(const model::ControlIntent& src) const {
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
