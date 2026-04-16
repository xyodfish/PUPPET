#pragma once

#include <string>

#include "puppet/orchestrator/orchestrator.hpp"
#include "puppet/retargeting/core/retargeting_pipeline.hpp"
#include "puppet/robot/backend/direct_mapping_backend.hpp"
#include "puppet/runtime/runtime_config.hpp"
#include "puppet/source/source_manager.hpp"

namespace puppet::runtime {

    class TeleopRuntime {
       public:
        bool init(const std::string& configPath, std::string* error);
        bool runOnce(std::string* error);

        source::SourceManager* sourceManager() { return &sourceManager_; }
        const RuntimeConfig& config() const { return config_; }
        const model::ControlIntent& lastControlIntent() const { return lastControlIntent_; }

       private:
        RuntimeConfig config_;
        source::SourceManager sourceManager_;
        orchestrator::Orchestrator orchestrator_;
        retargeting::RetargetingPipeline pipeline_;
        robot::DirectMappingBackend backend_;
        uint64_t sequenceId_ = 0;
        model::ControlIntent lastControlIntent_;
    };

}  // namespace puppet::runtime
