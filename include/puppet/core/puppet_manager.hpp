#pragma once

#include <memory>
#include <string>

#include "puppet/config/puppet_config.hpp"
#include "puppet/runtime/robot_state_sync.hpp"
#include "puppet/runtime/runtime_state_report.hpp"
#include "puppet/runtime/teleop_runtime.hpp"
#include "puppet/transport/embosa_runtime_channel.hpp"

namespace puppet::runtime {

    enum class PuppetManagerState {
        kCreated,
        kConfigLoaded,
        kModulesInitialized,
        kRunning,
        kStopped,
        kError,
    };

    class PuppetManager {
       public:
        PuppetManager();

        bool init(const std::string& runtimeConfigPath, std::string& error);
        void run(std::string& error);
        void stop();
        PuppetManagerState state() const { return state_; }
        const std::string& lastError() const { return lastError_; }

       private:
        bool loadConfig(const std::string& runtimeConfigPath, std::string& error);
        bool initModules(std::string& error);
        bool processOneLoop(std::string& error);
        void setError(const std::string& error);

        PuppetConfig config_;
        std::unique_ptr<TeleopRuntime> runtime_;
        std::unique_ptr<EmbosaRuntimeChannel> channel_;
        std::unique_ptr<RuntimeStateReport> report_;
        std::shared_ptr<RobotStateSync> robotStateSync_;
        bool initialized_         = false;
        bool stopRequested_       = false;
        PuppetManagerState state_ = PuppetManagerState::kCreated;
        std::string lastError_;
    };

}  // namespace puppet::runtime
