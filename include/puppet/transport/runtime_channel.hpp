#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "puppet/control/control_intent_types.hpp"
#include "puppet/primitive/primitive_types.hpp"

namespace puppet::runtime {

    class IRuntimeChannel {
       public:
        using PrimitiveFrameHandler  = std::function<void(const model::PrimitiveFrame&)>;
        using RobotStateFrameHandler = std::function<void(const model::PrimitiveFrame&)>;

        struct RuntimeStats {
            uint64_t receivedPrimitiveFrameCount  = 0;
            uint64_t receivedRobotStateFrameCount = 0;
            uint64_t droppedPrimitiveFrameCount   = 0;
            uint64_t publishedControlIntentCount  = 0;
            std::string lastError;
        };

        virtual ~IRuntimeChannel() = default;

        virtual bool start(std::string& error)                                                    = 0;
        virtual bool isRunning() const                                                            = 0;
        virtual bool tryPopFrame(model::PrimitiveFrame& frame)                                    = 0;
        virtual bool publishControlIntent(const model::ControlIntent& intent, std::string& error) = 0;
        virtual void registerPrimitiveFrameHandler(PrimitiveFrameHandler handler)                 = 0;
        virtual void registerRobotStateFrameHandler(RobotStateFrameHandler handler)               = 0;
        virtual RuntimeStats getRuntimeStats() const                                              = 0;
        virtual int idleSleepMs() const                                                           = 0;
    };

}  // namespace puppet::runtime
