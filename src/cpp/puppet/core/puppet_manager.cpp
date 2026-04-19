#include "puppet/core/puppet_manager.hpp"

#include <chrono>
#include <thread>

#include <glog/logging.h>

#include "puppet/common/time_utils.hpp"

namespace puppet::runtime {

    PuppetManager::PuppetManager() {
        runtime_ = std::make_unique<TeleopRuntime>();
        channel_ = std::make_unique<EmbosaRuntimeChannel>();
        report_  = std::make_unique<RuntimeStateReport>(500);
    }

    bool PuppetManager::init(const std::string& runtimeConfigPath, std::string& error) {
        stopRequested_ = false;

        if (!loadConfig(runtimeConfigPath, error)) {
            setError(error);
            LOG(ERROR) << "PuppetManager load config failed: " << error;
            return false;
        }

        if (!initModules(error)) {
            setError(error);
            LOG(ERROR) << "PuppetManager init modules failed: " << error;
            return false;
        }

        initialized_ = true;
        state_       = PuppetManagerState::kModulesInitialized;
        lastError_.clear();
        error.clear();
        return true;
    }

    void PuppetManager::run(std::string& error) {
        if (!initialized_) {
            error = "PuppetManager is not initialized";
            setError(error);
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
        error.clear();
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

        if (!channel_->start(config_.embosaRuntime, error)) {
            return false;
        }

        return true;
    }

    bool PuppetManager::processOneLoop(std::string& error) {
        model::PrimitiveFrame frame;
        if (!channel_->tryPopFrame(frame)) {
            report_->recordNoFrameSleep();
            error.clear();
            return true;
        }

        runtime_->sourceManager()->captureFrame(frame);
        if (!runtime_->runOnce(error)) {
            report_->recordRunOnceFailure(error);
            LOG(ERROR) << "PuppetManager runOnce failed: " << error;
            return true;
        }

        if (!channel_->publishControlIntent(runtime_->lastControlIntent(), error)) {
            report_->recordPublishFailure(error);
            LOG(ERROR) << "PuppetManager publish control intent failed: " << error;
            return true;
        }

        report_->recordSuccessfulLoop();
        report_->maybeReport(*runtime_, *channel_);
        error.clear();
        return true;
    }

    void PuppetManager::setError(const std::string& error) {
        state_     = PuppetManagerState::kError;
        lastError_ = error;
    }

}  // namespace puppet::runtime
