#include "puppet/core/puppet_manager.hpp"

#include <chrono>
#include <thread>

#include <glog/logging.h>

#include "puppet/common/logging.hpp"
#include "puppet/common/time_utils.hpp"
#include "puppet/transport/embosa/embosa_runtime_channel.hpp"
#include "puppet/transport/tcp/tcp_runtime_channel.hpp"
#include "puppet/transport/udp/udp_runtime_channel.hpp"
#include "puppet/transport/zmq/zmq_runtime_channel.hpp"

namespace puppet::runtime {

    PuppetManager::PuppetManager() {
        runtime_ = std::make_unique<TeleopRuntime>();
        report_  = std::make_unique<RuntimeStateReport>(500);
    }

    bool PuppetManager::init(const std::string& runtimeConfigPath, std::string& error) {
        stopRequested_ = false;

        if (!loadConfig(runtimeConfigPath, error)) {
            setError(error);
            PUPPET_LOG(ERROR, "config_load_failed", "puppet_manager", "init") << " config_path=" << runtimeConfigPath << " error=" << error;
            return false;
        }

        if (!initModules(error)) {
            setError(error);
            PUPPET_LOG(ERROR, "modules_init_failed", "puppet_manager", "init") << " error=" << error;
            return false;
        }

        initialized_ = true;
        state_       = PuppetManagerState::kModulesInitialized;
        lastError_.clear();
        common::ClearError(error);
        PUPPET_LOG(INFO, "manager_initialized", "puppet_manager", "init")
            << " config_path=" << runtimeConfigPath << " channel=" << config_.runtimeChannelType;
        return true;
    }

    void PuppetManager::run(std::string& error) {
        if (!initialized_) {
            error = "PuppetManager is not initialized";
            setError(error);
            PUPPET_LOG(ERROR, "run_rejected", "puppet_manager", "run") << " error=" << error;
            return;
        }

        state_ = PuppetManagerState::kRunning;
        while (!stopRequested_ && channel_->isRunning()) {
            const auto tStart = std::chrono::steady_clock::now();
            if (!processOneLoop(error)) {
                setError(error);
                return;
            }

            sysPreciseDelay(tStart, static_cast<double>(channel_->idleSleepMs()) * 1e-3);
        }

        state_ = PuppetManagerState::kStopped;
        common::ClearError(error);
        PUPPET_LOG(INFO, "manager_stopped", "puppet_manager", "run");
    }

    void PuppetManager::stop() {
        stopRequested_ = true;
    }

    bool PuppetManager::loadConfig(const std::string& runtimeConfigPath, std::string& error) {
        if (!PuppetConfigLoader::loadFromYamlFile(runtimeConfigPath, config_, error)) {
            return false;
        }
        state_ = PuppetManagerState::kConfigLoaded;
        return true;
    }

    bool PuppetManager::initModules(std::string& error) {
        if (!createRuntimeChannel(error)) {
            return false;
        }

        robotStateSync_ = std::make_shared<RobotStateSync>();
        runtime_->setRobotStateSync(robotStateSync_);
        channel_->registerRobotStateFrameHandler([this](const model::PrimitiveFrame& frame) {
            if (robotStateSync_ != nullptr) {
                robotStateSync_->update(frame);
            }
        });

        if (!runtime_->init(config_.runtime, error)) {
            return false;
        }

        if (!channel_->start(error)) {
            return false;
        }

        return true;
    }

    bool PuppetManager::createRuntimeChannel(std::string& error) {
        if (config_.runtimeChannelType == "embosa") {
            channel_ = std::make_unique<EmbosaRuntimeChannel>(config_.embosaRuntime);
            common::ClearError(error);
            PUPPET_LOG(INFO, "channel_created", "puppet_manager", "create_runtime_channel") << " channel=embosa";
            return true;
        }
        if (config_.runtimeChannelType == "zmq") {
            channel_ = std::make_unique<ZmqRuntimeChannel>(config_.zmqRuntime);
            common::ClearError(error);
            PUPPET_LOG(INFO, "channel_created", "puppet_manager", "create_runtime_channel") << " channel=zmq";
            return true;
        }
        if (config_.runtimeChannelType == "tcp") {
            channel_ = std::make_unique<TcpRuntimeChannel>(config_.tcpRuntime);
            common::ClearError(error);
            PUPPET_LOG(INFO, "channel_created", "puppet_manager", "create_runtime_channel") << " channel=tcp";
            return true;
        }
        if (config_.runtimeChannelType == "udp") {
            channel_ = std::make_unique<UdpRuntimeChannel>(config_.udpRuntime);
            common::ClearError(error);
            PUPPET_LOG(INFO, "channel_created", "puppet_manager", "create_runtime_channel") << " channel=udp";
            return true;
        }

        return common::Fail(error, "unsupported runtime channel type: " + config_.runtimeChannelType);
    }

    bool PuppetManager::processOneLoop(std::string& error) {
        model::PrimitiveFrame frame;
        if (!channel_->tryPopFrame(frame)) {
            report_->recordNoFrameSleep();
            common::ClearError(error);
            return true;
        }

        runtime_->sourceManager()->captureFrame(frame);
        if (!runtime_->runOnce(error)) {
            report_->recordRunOnceFailure(error);
            PUPPET_LOG_EVERY_N(ERROR, 100, "run_once_failed", "puppet_manager", "process_one_loop")
                << " source_id=" << frame.context.sourceId << " pipeline_id=" << frame.context.pipelineId
                << " seq=" << runtime_->lastControlIntent().sequenceId << " error=" << error;
            return true;
        }

        if (!channel_->publishControlIntent(runtime_->lastControlIntent(), error)) {
            report_->recordPublishFailure(error);
            PUPPET_LOG_EVERY_N(ERROR, 100, "publish_failed", "puppet_manager", "process_one_loop")
                << " seq=" << runtime_->lastControlIntent().sequenceId << " error=" << error;
            return true;
        }

        report_->recordSuccessfulLoop();
        report_->maybeReport(*runtime_, *channel_);
        common::ClearError(error);
        return true;
    }

    void PuppetManager::setError(const std::string& error) {
        state_     = PuppetManagerState::kError;
        lastError_ = error;
    }

}  // namespace puppet::runtime
