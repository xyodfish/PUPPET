#pragma once

#include "puppet/common/base_types.hpp"
#include "puppet/common/uniform_types.hpp"
#include "puppet/control/control_intent_types.hpp"
#include "puppet/primitive/primitive_trajectory_types.hpp"
#include "puppet/primitive/primitive_types.hpp"

#include "puppet/common.pb.h"
#include "puppet/control_intent.pb.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/primitive_trajectory.pb.h"
#include "puppet/uniform.pb.h"

namespace puppet::transport {

    bool copyToProto(const model::PrimitiveFrame& src, ::puppet::puppet_proto::PrimitiveFrame* dst);

    // Basic types.
    bool copyFromProto(const ::puppet::puppet_proto::Timestamp& src, model::Timestamp* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Duration& src, model::Duration* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Header& src, model::Header* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Point& src, model::Point* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Vector3& src, model::Vector3* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Quaternion& src, model::Quaternion* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Pose& src, model::Pose* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Twist& src, model::Twist* dst);

    // Uniform types.
    bool copyFromProto(const ::puppet::puppet_proto::FrameContext& src, model::FrameContext* dst);
    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveMeta& src, model::PrimitiveMeta* dst);

    // Primitive frame and sub-types.
    bool copyFromProto(const ::puppet::puppet_proto::PosePrimitive& src, model::PosePrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::TwistPrimitive& src, model::TwistPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::JointStatePrimitive& src, model::JointStatePrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::JointCommandPrimitive& src, model::JointCommandPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::ScalarPrimitive& src, model::ScalarPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::BooleanPrimitive& src, model::BooleanPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::ModePrimitive& src, model::ModePrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::PlanarMotionPrimitive& src, model::PlanarMotionPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::SkeletonJoint& src, model::SkeletonJoint* dst);
    bool copyFromProto(const ::puppet::puppet_proto::SkeletonPrimitive& src, model::SkeletonPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::Landmark& src, model::Landmark* dst);
    bool copyFromProto(const ::puppet::puppet_proto::LandmarkSetPrimitive& src, model::LandmarkSetPrimitive* dst);
    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveFrame& src, model::PrimitiveFrame* dst);

    // Control intent and sub-types.
    bool copyFromProto(const ::puppet::puppet_proto::CartPoseIntent& src, model::CartPoseIntent* dst);
    bool copyFromProto(const ::puppet::puppet_proto::JointCommandIntent& src, model::JointCommandIntent* dst);
    bool copyFromProto(const ::puppet::puppet_proto::PostureIntent& src, model::PostureIntent* dst);
    bool copyFromProto(const ::puppet::puppet_proto::BaseMotionIntent& src, model::BaseMotionIntent* dst);
    bool copyFromProto(const ::puppet::puppet_proto::ConstraintRequest& src, model::ConstraintRequest* dst);
    bool copyFromProto(const ::puppet::puppet_proto::GroupControlIntent& src, model::GroupControlIntent* dst);
    bool copyFromProto(const ::puppet::puppet_proto::ControlIntent& src, model::ControlIntent* dst);

    // Trajectory and sub-types.
    bool copyFromProto(const ::puppet::puppet_proto::TrajectoryHeader& src, model::TrajectoryHeader* dst);
    bool copyFromProto(const ::puppet::puppet_proto::TimedPrimitiveFrame& src, model::TimedPrimitiveFrame* dst);
    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveFrameTrajectory& src, model::PrimitiveFrameTrajectory* dst);

}  // namespace puppet::transport
