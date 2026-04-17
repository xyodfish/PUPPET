#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "puppet/common/base_types.hpp"
#include "puppet/common/uniform_types.hpp"

namespace puppet::model {

    struct CartPoseIntent {
        std::string eeName;
        std::string referenceFrameId;
        Pose pose;
        Twist twistFeedforward;
        double positionWeight    = 0.0;
        double orientationWeight = 0.0;
        float confidence         = 0.0F;
    };

    struct JointCommandIntent {
        enum class Mode {
            kUnspecified      = 0,
            kPosition         = 1,
            kVelocity         = 2,
            kEffort           = 3,
            kPositionVelocity = 4,
            kPositionEffort   = 5,
        };

        BodyGroup bodyGroup = BodyGroup::kUnspecified;
        Mode mode           = Mode::kUnspecified;
        std::vector<std::string> jointNames;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> effort;
        std::vector<double> stiffness;
        std::vector<double> damping;
        double weight = 0.0;
    };

    struct PostureIntent {
        BodyGroup bodyGroup = BodyGroup::kUnspecified;
        std::vector<std::string> jointNames;
        std::vector<double> preferredPosition;
        double weight = 0.0;
    };

    struct BaseMotionIntent {
        std::string referenceFrameId;
        double vx         = 0.0;
        double vy         = 0.0;
        double wz         = 0.0;
        double accelLimit = 0.0;
    };

    struct ConstraintRequest {
        enum class Type {
            kUnspecified                   = 0,
            kJointPositionLimit            = 1,
            kJointVelocityLimit            = 2,
            kJointAccelerationLimit        = 3,
            kSelfCollisionAvoidance        = 4,
            kEnvironmentCollisionAvoidance = 5,
            kComSupport                    = 6,
            kCustom                        = 100,
        };

        Type type     = Type::kUnspecified;
        bool hard     = false;
        double weight = 0.0;
        std::string target;
        std::unordered_map<std::string, double> scalarParams;
        std::unordered_map<std::string, std::string> stringParams;
    };

    struct GroupControlIntent {
        BodyGroup bodyGroup = BodyGroup::kUnspecified;
        std::string ownerSourceId;
        std::string mode;
        uint32_t priority = 0;
        std::string backendHint;
        bool enabled = false;

        std::vector<CartPoseIntent> eePoseIntents;
        std::vector<JointCommandIntent> jointCommandIntents;
        std::vector<PostureIntent> postureIntents;
        std::vector<BaseMotionIntent> baseMotionIntents;
        std::vector<ConstraintRequest> constraints;

        TagMap tags;
    };

    struct ControlIntent {
        Header header;
        FrameContext context;
        uint64_t sequenceId = 0;
        std::vector<GroupControlIntent> groupIntents;
        TagMap tags;
    };

}  // namespace puppet::model
