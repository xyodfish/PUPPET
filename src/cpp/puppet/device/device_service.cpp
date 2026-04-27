#include "puppet/device/device_service.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace puppet::device {

    bool DeviceService::initialize(const DeviceServiceConfig& config, std::string* error) {
        config_ = config;

        provider_ = createDeviceProvider(config_.deviceType, error);
        if (provider_ == nullptr) {
            return false;
        }

        if (!provider_->initialize(config_.deviceNode, config_.frameId, config_.sourceId, error)) {
            return false;
        }

        channel_ = transport::createDeviceOutputChannel(config_.channelType, error);
        if (channel_ == nullptr) {
            return false;
        }

        if (!channel_->initialize(config_.nodeName, config_.topicName, config_.outputEndpoint, config_.channelNode, error)) {
            return false;
        }

        if (config_.loopHz <= 0) {
            config_.loopHz = provider_->suggestedLoopHz();
        }
        if (config_.loopHz <= 0) {
            config_.loopHz = 50;
        }
        return true;
    }

    int DeviceService::run() {
        const auto sleepDuration = std::chrono::milliseconds(1000 / (config_.loopHz > 0 ? config_.loopHz : 50));

        uint64_t sequenceId = 0;
        while (channel_->ok()) {
            model::PrimitiveFrame frame;
            std::string error;
            if (provider_->nextFrame(sequenceId, &frame, &error)) {
                if (!channel_->publish(frame, &error)) {
                    std::cerr << "[device_service] publish failed: " << error << std::endl;
                    break;
                }

                if ((sequenceId % 25U) == 0U) {
                    std::cout << "[device_service] seq=" << sequenceId << " poses=" << frame.poses.size()
                              << " joints=" << frame.jointStates.size() << std::endl;
                }
                ++sequenceId;
            } else if (!error.empty()) {
                std::cerr << "[device_service] provider warning: " << error << std::endl;
            }

            std::this_thread::sleep_for(sleepDuration);
        }

        channel_->shutdown();
        return 0;
    }

}  // namespace puppet::device
