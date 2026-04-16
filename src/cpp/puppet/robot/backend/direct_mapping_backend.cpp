#include "puppet/robot/backend/direct_mapping_backend.hpp"

namespace puppet::robot {

    namespace {
        std::string toBodyGroupString(model::BodyGroup bodyGroup) {
            switch (bodyGroup) {
                case model::BodyGroup::kHead:
                    return "head";
                case model::BodyGroup::kLeftArm:
                    return "left_arm";
                case model::BodyGroup::kRightArm:
                    return "right_arm";
                case model::BodyGroup::kBase:
                    return "base";
                case model::BodyGroup::kWholeBody:
                    return "whole_body";
                default:
                    return "custom";
            }
        }
    }  // namespace

    target::FinalTarget DirectMappingBackend::buildTarget(const model::ControlIntent& controlIntent) const {
        target::FinalTarget target;
        target.sequenceId = controlIntent.sequenceId;

        for (const auto& groupIntent : controlIntent.groupIntents) {
            target::GroupTarget groupTarget;
            groupTarget.bodyGroup = toBodyGroupString(groupIntent.bodyGroup);

            if (!groupIntent.jointCommandIntents.empty()) {
                const auto& jointIntent   = groupIntent.jointCommandIntents.front();
                groupTarget.jointNames    = jointIntent.jointNames;
                groupTarget.jointPosition = jointIntent.position;
            }

            if (!groupIntent.baseMotionIntents.empty()) {
                const auto& baseIntent = groupIntent.baseMotionIntents.front();
                groupTarget.vx         = baseIntent.vx;
                groupTarget.vy         = baseIntent.vy;
                groupTarget.wz         = baseIntent.wz;
            }

            target.groups.push_back(std::move(groupTarget));
        }

        return target;
    }

}  // namespace puppet::robot
