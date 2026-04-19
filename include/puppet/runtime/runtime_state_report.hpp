#pragma once

#include <cstdint>
#include <string>

#include "puppet/runtime/teleop_runtime.hpp"
#include "puppet/transport/embosa_runtime_channel.hpp"

namespace puppet::runtime {

    class RuntimeStateReport {
       public:
        explicit RuntimeStateReport(uint64_t reportIntervalLoops = 500) : reportIntervalLoops_(reportIntervalLoops) {}

        void recordNoFrameSleep();
        void recordRunOnceFailure(const std::string& error);
        void recordPublishFailure(const std::string& error);
        void recordSuccessfulLoop();

        void maybeReport(const TeleopRuntime& runtime, const EmbosaRuntimeChannel& channel);

       private:
        uint64_t reportIntervalLoops_ = 500;
        uint64_t loopCount_           = 0;
        uint64_t noFrameSleepCount_   = 0;
        uint64_t runOnceFailCount_    = 0;
        uint64_t publishFailCount_    = 0;
        std::string lastRunOnceError_;
        std::string lastPublishError_;
    };

}  // namespace puppet::runtime
