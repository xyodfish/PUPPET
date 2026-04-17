#pragma once

#include <chrono>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "puppet/primitive/primitive_types.hpp"

namespace puppet::runtime {

    class RobotStateSync {
       public:
        struct Snapshot {
            bool hasState = false;
            model::PrimitiveFrame frame;
            std::unordered_map<std::string, double> jointPositionMap;
            std::chrono::steady_clock::time_point updatedAt;
        };

        void update(const model::PrimitiveFrame& frame);
        bool hasFreshState(int32_t freshnessTimeoutMs) const;
        Snapshot snapshot() const;

       private:
        static std::unordered_map<std::string, double> buildJointPositionMap(const model::PrimitiveFrame& frame);

        mutable std::mutex mutex_;
        Snapshot snapshot_;
    };

}  // namespace puppet::runtime
