#include "puppet/source/source_manager.hpp"

#include <atomic>

#include <glog/logging.h>

namespace puppet::source {

    void SourceManager::configure(const std::vector<runtime::SourceConfig>& sourceConfigs) {
        states_.clear();
        for (const auto& sourceConfig : sourceConfigs) {
            SourceState state;
            state.config = sourceConfig;
            states_.emplace(sourceConfig.sourceId, std::move(state));
            snapshotManager_.setMaxHistorySize(sourceConfig.topicName, static_cast<size_t>(sourceConfig.historySize));
        }
    }

    void SourceManager::ingestFrame(const model::PrimitiveFrame& frame) {
        auto stateIt = states_.find(frame.context.sourceId);
        if (stateIt == states_.end()) {
            static std::atomic<uint64_t> unknownSourceCount{0};
            const uint64_t count = ++unknownSourceCount;
            if ((count % 200ULL) == 1ULL) {
                LOG(WARNING) << "SourceManager ignore frame from unknown source_id=" << frame.context.sourceId
                             << " known_source_count=" << states_.size() << " ignored_count=" << count;
            }
            return;
        }
        auto framePtr               = std::make_shared<model::PrimitiveFrame>(frame);
        stateIt->second.latestFrame = framePtr;
        stateIt->second.lastReceive = std::chrono::steady_clock::now();
        snapshotManager_.updateMessage<model::PrimitiveFrame>(stateIt->second.config.topicName, framePtr);
    }

    std::shared_ptr<model::PrimitiveFrame> SourceManager::getLatestFrame(const std::string& sourceId) const {
        auto stateIt = states_.find(sourceId);
        if (stateIt == states_.end()) {
            return nullptr;
        }
        return stateIt->second.latestFrame;
    }

    SourceHealth SourceManager::getSourceHealth(const std::string& sourceId) const {
        SourceHealth output;
        auto stateIt = states_.find(sourceId);
        if (stateIt == states_.end()) {
            return output;
        }

        if (stateIt->second.lastReceive == std::chrono::steady_clock::time_point::min()) {
            return output;
        }

        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - stateIt->second.lastReceive);
        output.ageMs   = elapsed.count();
        output.healthy = output.ageMs <= stateIt->second.config.freshnessTimeoutMs;
        return output;
    }

}  // namespace puppet::source
