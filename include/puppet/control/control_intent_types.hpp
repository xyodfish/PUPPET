#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "puppet/common/base_types.hpp"
#include "puppet/common/uniform_types.hpp"

namespace puppet::model {

struct CartPoseIntent {
  std::string ee_name;
  std::string reference_frame_id;
  Pose pose;
  Twist twist_feedforward;
  double position_weight = 0.0;
  double orientation_weight = 0.0;
  float confidence = 0.0F;
};

struct JointCommandIntent {
  enum class Mode {
    kUnspecified = 0,
    kPosition = 1,
    kVelocity = 2,
    kEffort = 3,
    kPositionVelocity = 4,
    kPositionEffort = 5,
  };

  BodyGroup body_group = BodyGroup::kUnspecified;
  Mode mode = Mode::kUnspecified;
  std::vector<std::string> joint_names;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> stiffness;
  std::vector<double> damping;
  double weight = 0.0;
};

struct PostureIntent {
  BodyGroup body_group = BodyGroup::kUnspecified;
  std::vector<std::string> joint_names;
  std::vector<double> preferred_position;
  double weight = 0.0;
};

struct BaseMotionIntent {
  std::string reference_frame_id;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  double accel_limit = 0.0;
};

struct ConstraintRequest {
  enum class Type {
    kUnspecified = 0,
    kJointPositionLimit = 1,
    kJointVelocityLimit = 2,
    kJointAccelerationLimit = 3,
    kSelfCollisionAvoidance = 4,
    kEnvironmentCollisionAvoidance = 5,
    kComSupport = 6,
    kCustom = 100,
  };

  Type type = Type::kUnspecified;
  bool hard = false;
  double weight = 0.0;
  std::string target;
  std::unordered_map<std::string, double> scalar_params;
  std::unordered_map<std::string, std::string> string_params;
};

struct GroupControlIntent {
  BodyGroup body_group = BodyGroup::kUnspecified;
  std::string owner_source_id;
  std::string mode;
  uint32_t priority = 0;
  std::string backend_hint;
  bool enabled = false;

  std::vector<CartPoseIntent> ee_pose_intents;
  std::vector<JointCommandIntent> joint_command_intents;
  std::vector<PostureIntent> posture_intents;
  std::vector<BaseMotionIntent> base_motion_intents;
  std::vector<ConstraintRequest> constraints;

  TagMap tags;
};

struct ControlIntent {
  Header header;
  FrameContext context;
  uint64_t sequence_id = 0;
  std::vector<GroupControlIntent> group_intents;
  TagMap tags;
};

}  // namespace puppet::model

