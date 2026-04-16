#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "puppet/primitive/primitive_types.hpp"
#include "puppet/runtime/message_snapshot.hpp"
#include "puppet/runtime/runtime_config.hpp"

namespace puppet::source {

    struct SourceHealth {
        bool healthy  = false;
        int64_t ageMs = 0;
    };

    class SourceManager {
       public:
        void configure(const std::vector<runtime::SourceConfig>& sourceConfigs);
        void ingestFrame(const model::PrimitiveFrame& frame);

        std::shared_ptr<model::PrimitiveFrame> getLatestFrame(const std::string& sourceId) const;
        SourceHealth getSourceHealth(const std::string& sourceId) const;

        runtime::MessageSnapshot getSnapshot() { return snapshotManager_.getSnapshot(); }

       private:
        struct SourceState {
            runtime::SourceConfig config;
            std::shared_ptr<model::PrimitiveFrame> latestFrame;
            std::chrono::steady_clock::time_point lastReceive = std::chrono::steady_clock::time_point::min();
        };

        std::unordered_map<std::string, SourceState> states_;
        runtime::MessageSnapshotManager snapshotManager_;
    };

}  // namespace puppet::source
