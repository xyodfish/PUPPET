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
  bool isRelative = false;
  std::string targetFrameId;
};

struct TwistPrimitive {
  PrimitiveMeta meta;
  Twist twist;
  std::string bodyFrameId;
  std::string referenceFrameId;
};

struct JointStatePrimitive {
  PrimitiveMeta meta;
  std::vector<std::string> jointNames;
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
  std::vector<std::string> jointNames;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> stiffness;
  std::vector<double> damping;
};

struct ScalarPrimitive {
  PrimitiveMeta meta;
  double value = 0.0;
  double minValue = 0.0;
  double maxValue = 0.0;
};

struct BooleanPrimitive {
  PrimitiveMeta meta;
  bool value = false;
};

struct ModePrimitive {
  PrimitiveMeta meta;
  std::string modeName;
  int32_t modeId = 0;
  bool sticky = false;
};

struct PlanarMotionPrimitive {
  PrimitiveMeta meta;
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  std::string referenceFrameId;
};

struct SkeletonJoint {
  std::string name;
  int32_t parentIndex = -1;
  Pose pose;
  float confidence = 0.0F;
};

struct SkeletonPrimitive {
  PrimitiveMeta meta;
  std::string skeletonName;
  std::string referenceFrameId;
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
  std::string setName;
  std::string referenceFrameId;
  std::vector<Landmark> landmarks;
};

struct PrimitiveFrame {
  Header header;
  FrameContext context;
  uint64_t sequenceId = 0;

  std::vector<PosePrimitive> poses;
  std::vector<TwistPrimitive> twists;
  std::vector<JointStatePrimitive> jointStates;
  std::vector<JointCommandPrimitive> jointCommands;
  std::vector<ScalarPrimitive> scalars;
  std::vector<BooleanPrimitive> booleans;
  std::vector<ModePrimitive> modes;
  std::vector<PlanarMotionPrimitive> planarMotions;
  std::vector<SkeletonPrimitive> skeletons;
  std::vector<LandmarkSetPrimitive> landmarkSets;

  TagMap tags;
};

} // namespace puppet::model
