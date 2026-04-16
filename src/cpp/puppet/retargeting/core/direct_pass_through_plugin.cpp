#include "puppet/retargeting/core/direct_pass_through_plugin.hpp"

#include <algorithm>

#include "puppet/common/uniform_types.hpp"

namespace puppet::retargeting {
    namespace {

        model::BodyGroup toBodyGroup(const std::string& bodyGroup) {
            if (bodyGroup == "left_arm") {
                return model::BodyGroup::kLeftArm;
            }
            if (bodyGroup == "right_arm") {
                return model::BodyGroup::kRightArm;
            }
            if (bodyGroup == "head") {
                return model::BodyGroup::kHead;
            }
            if (bodyGroup == "base") {
                return model::BodyGroup::kBase;
            }
            if (bodyGroup == "whole_body") {
                return model::BodyGroup::kWholeBody;
            }
            return model::BodyGroup::kCustom;
        }

    }  // namespace

    bool DirectPassThroughPlugin::configure(const runtime::RuntimeConfig&, std::string* error) {
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    bool DirectPassThroughPlugin::process(const model::PrimitiveFrame& input, const std::string& bodyGroup,
                                          model::GroupControlIntent* output, std::string* error) {
        if (output == nullptr) {
            if (error != nullptr) {
                *error = "output is null";
            }
            return false;
        }

        output->bodyGroup     = toBodyGroup(bodyGroup);
        output->ownerSourceId = input.context.sourceId;
        output->enabled       = true;

        for (const auto& posePrimitive : input.poses) {
            model::CartPoseIntent poseIntent;
            poseIntent.eeName            = posePrimitive.meta.entity;
            poseIntent.referenceFrameId  = posePrimitive.meta.referenceFrameId;
            poseIntent.pose              = posePrimitive.pose;
            poseIntent.confidence        = posePrimitive.meta.confidence;
            poseIntent.positionWeight    = 1.0;
            poseIntent.orientationWeight = 1.0;
            output->eePoseIntents.push_back(std::move(poseIntent));
        }

        for (const auto& jointPrimitive : input.jointCommands) {
            model::JointCommandIntent jointIntent;
            jointIntent.bodyGroup  = toBodyGroup(bodyGroup);
            jointIntent.jointNames = jointPrimitive.jointNames;
            jointIntent.position   = jointPrimitive.position;
            jointIntent.velocity   = jointPrimitive.velocity;
            jointIntent.effort     = jointPrimitive.effort;
            jointIntent.stiffness  = jointPrimitive.stiffness;
            jointIntent.damping    = jointPrimitive.damping;
            jointIntent.weight     = 1.0;
            output->jointCommandIntents.push_back(std::move(jointIntent));
        }

        for (const auto& planar : input.planarMotions) {
            model::BaseMotionIntent baseIntent;
            baseIntent.referenceFrameId = planar.referenceFrameId;
            baseIntent.vx               = planar.vx;
            baseIntent.vy               = planar.vy;
            baseIntent.wz               = planar.wz;
            output->baseMotionIntents.push_back(std::move(baseIntent));
        }

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

}  // namespace puppet::retargeting
