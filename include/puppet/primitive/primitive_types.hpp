#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "puppet/common/base_types.hpp"
#include "puppet/common/uniform_types.hpp"

namespace puppet::model {

struct PosePrimitive {
  PrimitiveMeta meta;
  Pose pose;
  bool is_relative = false;
  std::string target_frame_id;
};

struct TwistPrimitive {
  PrimitiveMeta meta;
  Twist twist;
  std::string body_frame_id;
  std::string reference_frame_id;
};

struct JointStatePrimitive {
  PrimitiveMeta meta;
  std::vector<std::string> joint_names;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> current;
};

enum class JointCommandMode {
  kUnspecified = 0,
  kPosition = 1,
  kVelocity = 2,
  kEffort = 3,
  kPositionVelocity = 4,
  kPositionEffort = 5,
};

struct JointCommandPrimitive {
  PrimitiveMeta meta;
  JointCommandMode mode = JointCommandMode::kUnspecified;
  std::vector<std::string> joint_names;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> stiffness;
  std::vector<double> damping;
};

struct ScalarPrimitive {
  PrimitiveMeta meta;
  double value = 0.0;
  double min_value = 0.0;
  double max_value = 0.0;
};

struct BooleanPrimitive {
  PrimitiveMeta meta;
  bool value = false;
};

struct ModePrimitive {
  PrimitiveMeta meta;
  std::string mode_name;
  int32_t mode_id = 0;
  bool sticky = false;
};

struct PlanarMotionPrimitive {
  PrimitiveMeta meta;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  std::string reference_frame_id;
};

struct SkeletonJoint {
  std::string name;
  int32_t parent_index = -1;
  Pose pose;
  float confidence = 0.0F;
};

struct SkeletonPrimitive {
  PrimitiveMeta meta;
  std::string skeleton_name;
  std::string reference_frame_id;
  std::vector<SkeletonJoint> joints;
};

struct Landmark {
  std::string name;
  Point position;
  float confidence = 0.0F;
  float visibility = 0.0F;
};

struct LandmarkSetPrimitive {
  PrimitiveMeta meta;
  std::string set_name;
  std::string reference_frame_id;
  std::vector<Landmark> landmarks;
};

struct PrimitiveFrame {
  Header header;
  FrameContext context;
  uint64_t sequence_id = 0;

  std::vector<PosePrimitive> poses;
  std::vector<TwistPrimitive> twists;
  std::vector<JointStatePrimitive> joint_states;
  std::vector<JointCommandPrimitive> joint_commands;
  std::vector<ScalarPrimitive> scalars;
  std::vector<BooleanPrimitive> booleans;
  std::vector<ModePrimitive> modes;
  std::vector<PlanarMotionPrimitive> planar_motions;
  std::vector<SkeletonPrimitive> skeletons;
  std::vector<LandmarkSetPrimitive> landmark_sets;

  TagMap tags;
};

}  // namespace puppet::model

