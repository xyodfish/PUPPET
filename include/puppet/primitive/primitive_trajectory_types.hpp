#pragma once

#include <string>
#include <vector>

#include "puppet/common/base_types.hpp"
#include "puppet/common/uniform_types.hpp"
#include "puppet/primitive/primitive_types.hpp"

namespace puppet::model {

enum class TimeDomain {
  kUnspecified = 0,
  kMonotonic = 1,
  kWallClock = 2,
  kRobotTime = 3,
};

enum class InterpolationMode {
  kUnspecified = 0,
  kHold = 1,
  kLinear = 2,
  kSlerp = 3,
};

struct TrajectoryHeader {
  Header header;
  FrameContext context;
  std::string trajectoryId;
  TimeDomain timeDomain = TimeDomain::kUnspecified;
  Duration totalDuration;
  bool looping = false;
  InterpolationMode defaultInterp = InterpolationMode::kUnspecified;
  TagMap tags;
};

struct TimedPrimitiveFrame {
  Duration tFromStart;
  PrimitiveFrame frame;
};

struct PrimitiveFrameTrajectory {
  TrajectoryHeader trajectory;
  std::vector<TimedPrimitiveFrame> samples;
  TagMap tags;
};

}  // namespace puppet::model
