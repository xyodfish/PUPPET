#include "puppet/transport/proto_copy.hpp"

#include <utility>

namespace puppet::transport {

    template <typename ProtoMap, typename DstMap>
    void CopyMap(const ProtoMap& src, DstMap* dst) {
        dst->clear();
        dst->reserve(src.size());
        for (const auto& kv : src) {
            dst->emplace(kv.first, kv.second);
        }
    }

    template <typename ProtoRepeated, typename DstVec>
    void CopyStringRepeated(const ProtoRepeated& src, DstVec* dst) {
        dst->assign(src.begin(), src.end());
    }

    template <typename ProtoRepeated, typename DstVec>
    void CopyScalarRepeated(const ProtoRepeated& src, DstVec* dst) {
        dst->assign(src.begin(), src.end());
    }

    namespace {

        template <typename SrcMap, typename DstMap>
        void CopyToProtoMap(const SrcMap& src, DstMap* dst) {
            dst->clear();
            for (const auto& kv : src) {
                (*dst)[kv.first] = kv.second;
            }
        }

        void CopyToProto(const model::Timestamp& src, ::puppet::puppet_proto::Timestamp* dst) {
            dst->set_sec(src.sec);
            dst->set_nanosec(src.nanosec);
        }

        void CopyToProto(const model::Header& src, ::puppet::puppet_proto::Header* dst) {
            CopyToProto(src.timestamp, dst->mutable_timestamp());
            dst->set_frame_id(src.frameId);
        }

        void CopyToProto(const model::Point& src, ::puppet::puppet_proto::Point* dst) {
            dst->set_x(src.x);
            dst->set_y(src.y);
            dst->set_z(src.z);
        }

        void CopyToProto(const model::Vector3& src, ::puppet::puppet_proto::Vector3* dst) {
            dst->set_x(src.x);
            dst->set_y(src.y);
            dst->set_z(src.z);
        }

        void CopyToProto(const model::Quaternion& src, ::puppet::puppet_proto::Quaternion* dst) {
            dst->set_x(src.x);
            dst->set_y(src.y);
            dst->set_z(src.z);
            dst->set_w(src.w);
        }

        void CopyToProto(const model::Pose& src, ::puppet::puppet_proto::Pose* dst) {
            CopyToProto(src.position, dst->mutable_position());
            CopyToProto(src.orientation, dst->mutable_orientation());
        }

        void CopyToProto(const model::Twist& src, ::puppet::puppet_proto::Twist* dst) {
            CopyToProto(src.linear, dst->mutable_linear());
            CopyToProto(src.angular, dst->mutable_angular());
        }

        void CopyToProto(const model::FrameContext& src, ::puppet::puppet_proto::FrameContext* dst) {
            dst->set_source_id(src.sourceId);
            dst->set_source_type(static_cast<::puppet::puppet_proto::SourceType>(src.sourceType));
            dst->set_semantic_context(src.semanticContext);
            dst->set_mode(src.mode);
            dst->set_robot_id(src.robotId);
            dst->set_pipeline_id(src.pipelineId);
            CopyToProtoMap(src.tags, dst->mutable_tags());
        }

        void CopyToProto(const model::PrimitiveMeta& src, ::puppet::puppet_proto::PrimitiveMeta* dst) {
            dst->set_name(src.name);
            dst->set_entity(src.entity);
            dst->set_body_group(static_cast<::puppet::puppet_proto::BodyGroup>(src.bodyGroup));
            dst->set_frame_id(src.frameId);
            dst->set_reference_frame_id(src.referenceFrameId);
            CopyToProto(src.timestamp, dst->mutable_timestamp());
            dst->set_confidence(src.confidence);
            dst->set_valid(src.valid);
            CopyToProtoMap(src.tags, dst->mutable_tags());
        }

        uint32_t BuildIsRelativeFlags(const std::array<bool, 5>& flags) {
            uint32_t out = 0;
            for (size_t i = 0; i < flags.size(); ++i) {
                if (flags[i]) {
                    out |= (1u << i);
                }
            }
            return out;
        }

    }  // namespace

    bool copyToProto(const model::PrimitiveFrame& src, ::puppet::puppet_proto::PrimitiveFrame* dst) {
        if (dst == nullptr) {
            return false;
        }

        dst->Clear();
        CopyToProto(src.header, dst->mutable_header());
        CopyToProto(src.context, dst->mutable_context());
        dst->set_sequence_id(src.sequenceId);

        for (const auto& pose : src.poses) {
            auto* out = dst->add_poses();
            CopyToProto(pose.meta, out->mutable_meta());
            CopyToProto(pose.pose, out->mutable_pose());
            out->set_is_relative(pose.isRelative);
            out->set_target_frame_id(pose.targetFrameId);
        }

        for (const auto& twist : src.twists) {
            auto* out = dst->add_twists();
            CopyToProto(twist.meta, out->mutable_meta());
            CopyToProto(twist.twist, out->mutable_twist());
            out->set_is_relative(twist.isRelative);
            out->set_body_frame_id(twist.bodyFrameId);
            out->set_reference_frame_id(twist.referenceFrameId);
        }

        for (const auto& jointState : src.jointStates) {
            auto* out = dst->add_joint_states();
            CopyToProto(jointState.meta, out->mutable_meta());
            for (const auto& name : jointState.jointNames)
                out->add_joint_names(name);
            for (const auto value : jointState.position)
                out->add_position(value);
            for (const auto value : jointState.velocity)
                out->add_velocity(value);
            for (const auto value : jointState.acceleration)
                out->add_acceleration(value);
            for (const auto value : jointState.effort)
                out->add_effort(value);
            for (const auto value : jointState.current)
                out->add_current(value);
            out->set_is_relative_flags(BuildIsRelativeFlags(jointState.isRelatived));
        }

        for (const auto& jointCommand : src.jointCommands) {
            auto* out = dst->add_joint_commands();
            CopyToProto(jointCommand.meta, out->mutable_meta());
            out->set_mode(static_cast<::puppet::puppet_proto::JointCommandPrimitive_JointCommandMode>(jointCommand.mode));
            for (const auto& name : jointCommand.jointNames)
                out->add_joint_names(name);
            for (const auto value : jointCommand.position)
                out->add_position(value);
            for (const auto value : jointCommand.velocity)
                out->add_velocity(value);
            for (const auto value : jointCommand.acceleration)
                out->add_acceleration(value);
            for (const auto value : jointCommand.effort)
                out->add_effort(value);
            for (const auto value : jointCommand.stiffness)
                out->add_stiffness(value);
            for (const auto value : jointCommand.damping)
                out->add_damping(value);
            out->set_is_relative_flags(BuildIsRelativeFlags(jointCommand.isRelatived));
        }

        for (const auto& scalar : src.scalars) {
            auto* out = dst->add_scalars();
            CopyToProto(scalar.meta, out->mutable_meta());
            out->set_value(scalar.value);
            out->set_min_value(scalar.minValue);
            out->set_max_value(scalar.maxValue);
            out->set_scale_value(scalar.scaleValue);
            out->set_offset_value(scalar.offsetValue);
        }

        for (const auto& booleanValue : src.booleans) {
            auto* out = dst->add_booleans();
            CopyToProto(booleanValue.meta, out->mutable_meta());
            out->set_value(booleanValue.value);
        }

        for (const auto& mode : src.modes) {
            auto* out = dst->add_modes();
            CopyToProto(mode.meta, out->mutable_meta());
            out->set_mode_name(mode.modeName);
            out->set_mode_id(mode.modeId);
            out->set_sticky(mode.sticky);
        }

        for (const auto& planarMotion : src.planarMotions) {
            auto* out = dst->add_planar_motions();
            CopyToProto(planarMotion.meta, out->mutable_meta());
            out->set_vx(planarMotion.vx);
            out->set_vy(planarMotion.vy);
            out->set_wz(planarMotion.wz);
            out->set_reference_frame_id(planarMotion.referenceFrameId);
        }

        for (const auto& skeleton : src.skeletons) {
            auto* out = dst->add_skeletons();
            CopyToProto(skeleton.meta, out->mutable_meta());
            out->set_skeleton_name(skeleton.skeletonName);
            out->set_reference_frame_id(skeleton.referenceFrameId);
            for (const auto& joint : skeleton.joints) {
                auto* outJoint = out->add_joints();
                outJoint->set_name(joint.name);
                outJoint->set_parent_index(joint.parentIndex);
                CopyToProto(joint.pose, outJoint->mutable_pose());
                outJoint->set_confidence(joint.confidence);
            }
        }

        for (const auto& landmarkSet : src.landmarkSets) {
            auto* out = dst->add_landmark_sets();
            CopyToProto(landmarkSet.meta, out->mutable_meta());
            out->set_set_name(landmarkSet.setName);
            out->set_reference_frame_id(landmarkSet.referenceFrameId);
            for (const auto& landmark : landmarkSet.landmarks) {
                auto* outLandmark = out->add_landmarks();
                outLandmark->set_name(landmark.name);
                CopyToProto(landmark.position, outLandmark->mutable_position());
                outLandmark->set_confidence(landmark.confidence);
                outLandmark->set_visibility(landmark.visibility);
            }
        }

        CopyToProtoMap(src.tags, dst->mutable_tags());
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Timestamp& src, model::Timestamp* dst) {
        if (dst == nullptr)
            return false;
        dst->sec     = src.sec();
        dst->nanosec = src.nanosec();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Duration& src, model::Duration* dst) {
        if (dst == nullptr)
            return false;
        dst->sec     = src.sec();
        dst->nanosec = src.nanosec();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Header& src, model::Header* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.timestamp(), &dst->timestamp);
        dst->frameId = src.frame_id();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Point& src, model::Point* dst) {
        if (dst == nullptr)
            return false;
        dst->x = src.x();
        dst->y = src.y();
        dst->z = src.z();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Vector3& src, model::Vector3* dst) {
        if (dst == nullptr)
            return false;
        dst->x = src.x();
        dst->y = src.y();
        dst->z = src.z();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Quaternion& src, model::Quaternion* dst) {
        if (dst == nullptr)
            return false;
        dst->x = src.x();
        dst->y = src.y();
        dst->z = src.z();
        dst->w = src.w();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Pose& src, model::Pose* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.position(), &dst->position);
        copyFromProto(src.orientation(), &dst->orientation);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Twist& src, model::Twist* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.linear(), &dst->linear);
        copyFromProto(src.angular(), &dst->angular);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::FrameContext& src, model::FrameContext* dst) {
        if (dst == nullptr)
            return false;
        dst->sourceId        = src.source_id();
        dst->sourceType      = static_cast<model::SourceType>(src.source_type());
        dst->semanticContext = src.semantic_context();
        dst->mode            = src.mode();
        dst->robotId         = src.robot_id();
        dst->pipelineId      = src.pipeline_id();
        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveMeta& src, model::PrimitiveMeta* dst) {
        if (dst == nullptr)
            return false;
        dst->name             = src.name();
        dst->entity           = src.entity();
        dst->bodyGroup        = static_cast<model::BodyGroup>(src.body_group());
        dst->frameId          = src.frame_id();
        dst->referenceFrameId = src.reference_frame_id();
        copyFromProto(src.timestamp(), &dst->timestamp);
        dst->confidence = src.confidence();
        dst->valid      = src.valid();
        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PosePrimitive& src, model::PosePrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        copyFromProto(src.pose(), &dst->pose);
        dst->isRelative    = src.is_relative();
        dst->targetFrameId = src.target_frame_id();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::TwistPrimitive& src, model::TwistPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        copyFromProto(src.twist(), &dst->twist);
        dst->isRelative       = src.is_relative();
        dst->bodyFrameId      = src.body_frame_id();
        dst->referenceFrameId = src.reference_frame_id();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::JointStatePrimitive& src, model::JointStatePrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        CopyStringRepeated(src.joint_names(), &dst->jointNames);
        CopyScalarRepeated(src.position(), &dst->position);
        CopyScalarRepeated(src.velocity(), &dst->velocity);
        CopyScalarRepeated(src.effort(), &dst->effort);
        CopyScalarRepeated(src.current(), &dst->current);

        for (int i = 0; i < 5; ++i) {
            dst->isRelatived[i] = (src.is_relative_flags() >> i) & 1u;
        }
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::JointCommandPrimitive& src, model::JointCommandPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->mode = static_cast<model::JointCommandMode>(src.mode());
        CopyStringRepeated(src.joint_names(), &dst->jointNames);
        CopyScalarRepeated(src.position(), &dst->position);
        CopyScalarRepeated(src.velocity(), &dst->velocity);
        CopyScalarRepeated(src.effort(), &dst->effort);
        CopyScalarRepeated(src.stiffness(), &dst->stiffness);
        CopyScalarRepeated(src.damping(), &dst->damping);
        for (int i = 0; i < 5; ++i) {
            dst->isRelatived[i] = (src.is_relative_flags() >> i) & 1u;
        }
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::ScalarPrimitive& src, model::ScalarPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->value       = src.value();
        dst->minValue    = src.min_value();
        dst->maxValue    = src.max_value();
        dst->scaleValue  = src.scale_value();
        dst->offsetValue = src.offset_value();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::BooleanPrimitive& src, model::BooleanPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->value = src.value();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::ModePrimitive& src, model::ModePrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->modeName = src.mode_name();
        dst->modeId   = src.mode_id();
        dst->sticky   = src.sticky();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PlanarMotionPrimitive& src, model::PlanarMotionPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->vx               = src.vx();
        dst->vy               = src.vy();
        dst->wz               = src.wz();
        dst->referenceFrameId = src.reference_frame_id();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::SkeletonJoint& src, model::SkeletonJoint* dst) {
        if (dst == nullptr)
            return false;
        dst->name        = src.name();
        dst->parentIndex = src.parent_index();
        copyFromProto(src.pose(), &dst->pose);
        dst->confidence = src.confidence();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::SkeletonPrimitive& src, model::SkeletonPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->skeletonName     = src.skeleton_name();
        dst->referenceFrameId = src.reference_frame_id();
        dst->joints.clear();
        dst->joints.reserve(src.joints_size());
        for (const auto& joint : src.joints()) {
            model::SkeletonJoint copied;
            copyFromProto(joint, &copied);
            dst->joints.push_back(std::move(copied));
        }
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::Landmark& src, model::Landmark* dst) {
        if (dst == nullptr)
            return false;
        dst->name = src.name();
        copyFromProto(src.position(), &dst->position);
        dst->confidence = src.confidence();
        dst->visibility = src.visibility();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::LandmarkSetPrimitive& src, model::LandmarkSetPrimitive* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.meta(), &dst->meta);
        dst->setName          = src.set_name();
        dst->referenceFrameId = src.reference_frame_id();
        dst->landmarks.clear();
        dst->landmarks.reserve(src.landmarks_size());
        for (const auto& landmark : src.landmarks()) {
            model::Landmark copied;
            copyFromProto(landmark, &copied);
            dst->landmarks.push_back(std::move(copied));
        }
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveFrame& src, model::PrimitiveFrame* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.header(), &dst->header);
        copyFromProto(src.context(), &dst->context);
        dst->sequenceId = src.sequence_id();

        dst->poses.clear();
        dst->poses.reserve(src.poses_size());
        for (const auto& v : src.poses()) {
            model::PosePrimitive copied;
            copyFromProto(v, &copied);
            dst->poses.push_back(std::move(copied));
        }

        dst->twists.clear();
        dst->twists.reserve(src.twists_size());
        for (const auto& v : src.twists()) {
            model::TwistPrimitive copied;
            copyFromProto(v, &copied);
            dst->twists.push_back(std::move(copied));
        }

        dst->jointStates.clear();
        dst->jointStates.reserve(src.joint_states_size());
        for (const auto& v : src.joint_states()) {
            model::JointStatePrimitive copied;
            copyFromProto(v, &copied);
            dst->jointStates.push_back(std::move(copied));
        }

        dst->jointCommands.clear();
        dst->jointCommands.reserve(src.joint_commands_size());
        for (const auto& v : src.joint_commands()) {
            model::JointCommandPrimitive copied;
            copyFromProto(v, &copied);
            dst->jointCommands.push_back(std::move(copied));
        }

        dst->scalars.clear();
        dst->scalars.reserve(src.scalars_size());
        for (const auto& v : src.scalars()) {
            model::ScalarPrimitive copied;
            copyFromProto(v, &copied);
            dst->scalars.push_back(std::move(copied));
        }

        dst->booleans.clear();
        dst->booleans.reserve(src.booleans_size());
        for (const auto& v : src.booleans()) {
            model::BooleanPrimitive copied;
            copyFromProto(v, &copied);
            dst->booleans.push_back(std::move(copied));
        }

        dst->modes.clear();
        dst->modes.reserve(src.modes_size());
        for (const auto& v : src.modes()) {
            model::ModePrimitive copied;
            copyFromProto(v, &copied);
            dst->modes.push_back(std::move(copied));
        }

        dst->planarMotions.clear();
        dst->planarMotions.reserve(src.planar_motions_size());
        for (const auto& v : src.planar_motions()) {
            model::PlanarMotionPrimitive copied;
            copyFromProto(v, &copied);
            dst->planarMotions.push_back(std::move(copied));
        }

        dst->skeletons.clear();
        dst->skeletons.reserve(src.skeletons_size());
        for (const auto& v : src.skeletons()) {
            model::SkeletonPrimitive copied;
            copyFromProto(v, &copied);
            dst->skeletons.push_back(std::move(copied));
        }

        dst->landmarkSets.clear();
        dst->landmarkSets.reserve(src.landmark_sets_size());
        for (const auto& v : src.landmark_sets()) {
            model::LandmarkSetPrimitive copied;
            copyFromProto(v, &copied);
            dst->landmarkSets.push_back(std::move(copied));
        }

        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::CartPoseIntent& src, model::CartPoseIntent* dst) {
        if (dst == nullptr)
            return false;
        dst->eeName           = src.ee_name();
        dst->referenceFrameId = src.reference_frame_id();
        copyFromProto(src.pose(), &dst->pose);
        copyFromProto(src.twist_feedforward(), &dst->twistFeedforward);
        dst->positionWeight    = src.position_weight();
        dst->orientationWeight = src.orientation_weight();
        dst->confidence        = src.confidence();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::JointCommandIntent& src, model::JointCommandIntent* dst) {
        if (dst == nullptr)
            return false;
        dst->bodyGroup = static_cast<model::BodyGroup>(src.body_group());
        dst->mode      = static_cast<model::JointCommandIntent::Mode>(src.mode());
        CopyStringRepeated(src.joint_names(), &dst->jointNames);
        CopyScalarRepeated(src.position(), &dst->position);
        CopyScalarRepeated(src.velocity(), &dst->velocity);
        CopyScalarRepeated(src.effort(), &dst->effort);
        CopyScalarRepeated(src.stiffness(), &dst->stiffness);
        CopyScalarRepeated(src.damping(), &dst->damping);
        dst->weight = src.weight();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PostureIntent& src, model::PostureIntent* dst) {
        if (dst == nullptr)
            return false;
        dst->bodyGroup = static_cast<model::BodyGroup>(src.body_group());
        CopyStringRepeated(src.joint_names(), &dst->jointNames);
        CopyScalarRepeated(src.preferred_position(), &dst->preferredPosition);
        dst->weight = src.weight();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::BaseMotionIntent& src, model::BaseMotionIntent* dst) {
        if (dst == nullptr)
            return false;
        dst->referenceFrameId = src.reference_frame_id();
        dst->vx               = src.vx();
        dst->vy               = src.vy();
        dst->wz               = src.wz();
        dst->accelLimit       = src.accel_limit();
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::ConstraintRequest& src, model::ConstraintRequest* dst) {
        if (dst == nullptr)
            return false;
        dst->type   = static_cast<model::ConstraintRequest::Type>(src.type());
        dst->hard   = src.hard();
        dst->weight = src.weight();
        dst->target = src.target();
        CopyMap(src.scalar_params(), &dst->scalarParams);
        CopyMap(src.string_params(), &dst->stringParams);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::GroupControlIntent& src, model::GroupControlIntent* dst) {
        if (dst == nullptr)
            return false;
        dst->bodyGroup     = static_cast<model::BodyGroup>(src.body_group());
        dst->ownerSourceId = src.owner_source_id();
        dst->mode          = src.mode();
        dst->priority      = src.priority();
        dst->backendHint   = src.backend_hint();
        dst->enabled       = src.enabled();

        dst->eePoseIntents.clear();
        dst->eePoseIntents.reserve(src.ee_pose_intents_size());
        for (const auto& v : src.ee_pose_intents()) {
            model::CartPoseIntent copied;
            copyFromProto(v, &copied);
            dst->eePoseIntents.push_back(std::move(copied));
        }

        dst->jointCommandIntents.clear();
        dst->jointCommandIntents.reserve(src.joint_command_intents_size());
        for (const auto& v : src.joint_command_intents()) {
            model::JointCommandIntent copied;
            copyFromProto(v, &copied);
            dst->jointCommandIntents.push_back(std::move(copied));
        }

        dst->postureIntents.clear();
        dst->postureIntents.reserve(src.posture_intents_size());
        for (const auto& v : src.posture_intents()) {
            model::PostureIntent copied;
            copyFromProto(v, &copied);
            dst->postureIntents.push_back(std::move(copied));
        }

        dst->baseMotionIntents.clear();
        dst->baseMotionIntents.reserve(src.base_motion_intents_size());
        for (const auto& v : src.base_motion_intents()) {
            model::BaseMotionIntent copied;
            copyFromProto(v, &copied);
            dst->baseMotionIntents.push_back(std::move(copied));
        }

        dst->constraints.clear();
        dst->constraints.reserve(src.constraints_size());
        for (const auto& v : src.constraints()) {
            model::ConstraintRequest copied;
            copyFromProto(v, &copied);
            dst->constraints.push_back(std::move(copied));
        }

        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::ControlIntent& src, model::ControlIntent* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.header(), &dst->header);
        copyFromProto(src.context(), &dst->context);
        dst->sequenceId = src.sequence_id();

        dst->groupIntents.clear();
        dst->groupIntents.reserve(src.group_intents_size());
        for (const auto& v : src.group_intents()) {
            model::GroupControlIntent copied;
            copyFromProto(v, &copied);
            dst->groupIntents.push_back(std::move(copied));
        }

        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::TrajectoryHeader& src, model::TrajectoryHeader* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.header(), &dst->header);
        copyFromProto(src.context(), &dst->context);
        dst->trajectoryId = src.trajectory_id();
        dst->timeDomain   = static_cast<model::TimeDomain>(src.time_domain());
        copyFromProto(src.total_duration(), &dst->totalDuration);
        dst->looping       = src.looping();
        dst->defaultInterp = static_cast<model::InterpolationMode>(src.default_interp());
        CopyMap(src.tags(), &dst->tags);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::TimedPrimitiveFrame& src, model::TimedPrimitiveFrame* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.t_from_start(), &dst->tFromStart);
        copyFromProto(src.frame(), &dst->frame);
        return true;
    }

    bool copyFromProto(const ::puppet::puppet_proto::PrimitiveFrameTrajectory& src, model::PrimitiveFrameTrajectory* dst) {
        if (dst == nullptr)
            return false;
        copyFromProto(src.trajectory(), &dst->trajectory);

        dst->samples.clear();
        dst->samples.reserve(src.samples_size());
        for (const auto& v : src.samples()) {
            model::TimedPrimitiveFrame copied;
            copyFromProto(v, &copied);
            dst->samples.push_back(std::move(copied));
        }

        CopyMap(src.tags(), &dst->tags);
        return true;
    }

}  // namespace puppet::transport
