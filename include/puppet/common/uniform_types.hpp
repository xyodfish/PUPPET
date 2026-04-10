#pragma once

#include <string>
#include <unordered_map>

#include "puppet/common/base_types.hpp"

namespace puppet::model {

using TagMap = std::unordered_map<std::string, std::string>;

enum class BodyGroup {
  kUnspecified = 0,
  kHead = 1,
  kLeftArm = 2,
  kRightArm = 3,
  kBiManual = 4,
  kTorso = 5,
  kBase = 6,
  kLowerBody = 7,
  kWholeBody = 8,
  kLeftGripper = 9,
  kRightGripper = 10,
  kCustom = 100,
};

enum class SourceType {
  kUnspecified = 0,
  kVr = 1,
  kMasterArm = 2,
  kMocap = 3,
  kVision = 4,
  kJoystick = 5,
  kExternal = 100,
};

struct FrameContext {
  std::string sourceId;
  SourceType sourceType = SourceType::kUnspecified;
  std::string semanticContext;
  std::string mode;
  std::string robotId;
  std::string pipelineId;
  TagMap tags;
};

struct PrimitiveMeta {
  std::string name;
  std::string entity;
  BodyGroup bodyGroup = BodyGroup::kUnspecified;
  std::string frameId;
  std::string referenceFrameId;
  Timestamp timestamp;
  float confidence = 0.0F;
  bool valid = false;
  TagMap tags;
};

}  // namespace puppet::model
