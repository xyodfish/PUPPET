#include "puppet/runtime/robot_state_sync.hpp"

#include <algorithm>

namespace puppet::runtime {

    void RobotStateSync::update(const model::PrimitiveFrame& frame) {
        Snapshot next;
        next.hasState         = true;
        next.frame            = frame;
        next.jointPositionMap = buildJointPositionMap(frame);
        next.updatedAt        = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(next);
    }

    bool RobotStateSync::hasFreshState(int32_t freshnessTimeoutMs) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!snapshot_.hasState) {
            return false;
        }
        const auto now    = std::chrono::steady_clock::now();
        const auto ageMs  = std::chrono::duration_cast<std::chrono::milliseconds>(now - snapshot_.updatedAt).count();
        const auto maxAge = std::max<int64_t>(0, static_cast<int64_t>(freshnessTimeoutMs));
        return ageMs <= maxAge;
    }

    RobotStateSync::Snapshot RobotStateSync::snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

    std::unordered_map<std::string, double> RobotStateSync::buildJointPositionMap(const model::PrimitiveFrame& frame) {
        std::unordered_map<std::string, double> map;
        for (const auto& jointState : frame.jointStates) {
            const size_t size = std::min(jointState.jointNames.size(), jointState.position.size());
            for (size_t i = 0; i < size; ++i) {
                map[jointState.jointNames[i]] = jointState.position[i];
            }
        }
        return map;
    }

}  // namespace puppet::runtime
