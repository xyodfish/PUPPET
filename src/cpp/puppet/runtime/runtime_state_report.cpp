#include "puppet/runtime/runtime_state_report.hpp"

#include <glog/logging.h>

namespace puppet::runtime {

    void RuntimeStateReport::recordNoFrameSleep() {
        noFrameSleepCount_++;
    }

    void RuntimeStateReport::recordRunOnceFailure(const std::string& error) {
        runOnceFailCount_++;
        lastRunOnceError_ = error;
    }

    void RuntimeStateReport::recordPublishFailure(const std::string& error) {
        publishFailCount_ = publishFailCount_ + 1;
        lastPublishError_ = error;
    }

    void RuntimeStateReport::recordSuccessfulLoop() {
        loopCount_++;
    }

    void RuntimeStateReport::maybeReport(const TeleopRuntime& runtime, const EmbosaRuntimeChannel& channel) {
        if (reportIntervalLoops_ == 0 || loopCount_ == 0 || (loopCount_ % reportIntervalLoops_) != 0) {
            return;
        }

        uint64_t unhealthySources = 0;
        const auto& sources       = runtime.config().sources;
        for (const auto& source : sources) {
            const auto health = runtime.sourceManager()->getSourceHealth(source.sourceId);
            if (!health.healthy) {
                unhealthySources++;
            }
        }

        const auto channelStats = channel.getRuntimeStats();
        LOG(INFO) << "runtime report loop=" << loopCount_ << " no_frame_sleep=" << noFrameSleepCount_
                  << " run_once_fail=" << runOnceFailCount_ << " publish_fail=" << publishFailCount_
                  << " source_unhealthy=" << unhealthySources << "/" << sources.size() << " rx=" << channelStats.receivedPrimitiveFrameCount
                  << " rx_drop=" << channelStats.droppedPrimitiveFrameCount << " tx=" << channelStats.publishedControlIntentCount
                  << " channel_last_error=" << channelStats.lastError << " run_once_last_error=" << lastRunOnceError_
                  << " publish_last_error=" << lastPublishError_;
    }

}  // namespace puppet::runtime
