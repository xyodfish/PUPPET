#include "puppet/device/single_chain_ik/mixed_arm_split_demo_device_provider.hpp"

#include <chrono>
#include <cmath>
#include <vector>

namespace puppet::device {
    namespace {
        void resetFrame(model::PrimitiveFrame* frame) {
            frame->poses.clear();
            frame->twists.clear();
            frame->jointStates.clear();
            frame->jointCommands.clear();
            frame->scalars.clear();
            frame->booleans.clear();
            frame->modes.clear();
            frame->planarMotions.clear();
            frame->skeletons.clear();
            frame->landmarkSets.clear();
            frame->tags.clear();
        }

        void populateGroupPipelineIds(const std::string& leftPipelineId, const std::string& rightPipelineId, model::PrimitiveFrame* frame) {
            frame->context.groupPipelineIds.clear();
            if (!leftPipelineId.empty()) {
                frame->context.groupPipelineIds.emplace("left_arm", leftPipelineId);
            }
            if (!rightPipelineId.empty()) {
                frame->context.groupPipelineIds.emplace("right_arm", rightPipelineId);
            }
        }
    }  // namespace

    bool MixedArmSplitDemoDeviceProvider::initialize(const DeviceServiceConfig& config, std::string& error) {
        (void)error;
        frameId_                     = config.frameId;
        sourceId_                    = config.sourceId;
        const YAML::Node& configNode = config.deviceNode;

        if (configNode["loop_hz"]) {
            loopHz_ = configNode["loop_hz"].as<int>();
        }
        if (configNode["semantic_context"]) {
            semantic_ = configNode["semantic_context"].as<std::string>();
        }
        if (configNode["mode"]) {
            mode_ = configNode["mode"].as<std::string>();
        }
        if (configNode["left_pipeline_id"]) {
            leftPipelineId_ = configNode["left_pipeline_id"].as<std::string>();
        }
        if (configNode["right_pipeline_id"]) {
            rightPipelineId_ = configNode["right_pipeline_id"].as<std::string>();
        }
        if (configNode["robot_id"]) {
            robotId_ = configNode["robot_id"].as<std::string>();
        }
        if (loopHz_ <= 0) {
            loopHz_ = 50;
        }
        return true;
    }

    bool MixedArmSplitDemoDeviceProvider::nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string& error) {
        (void)error;
        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->header.timestamp.sec     = sec;
        frame->header.timestamp.nanosec = static_cast<uint32_t>(nsec);
        frame->header.frameId           = frameId_;
        frame->sequenceId               = sequenceId;
        frame->context.sourceId         = sourceId_;
        frame->context.sourceType       = model::SourceType::kExternal;
        frame->context.semanticContext  = semantic_;
        frame->context.mode             = mode_;
        frame->context.robotId          = robotId_;
        frame->context.pipelineId.clear();
        resetFrame(frame);

        const double dt    = 1.0 / static_cast<double>(loopHz_);
        const double t_sec = static_cast<double>(sequenceId) * dt;

        model::JointCommandPrimitive leftCommand;
        leftCommand.meta.name             = "left_arm_joint_command_demo";
        leftCommand.meta.entity           = "left_arm";
        leftCommand.meta.bodyGroup        = "left_arm";
        leftCommand.meta.frameId          = frameId_;
        leftCommand.meta.referenceFrameId = frameId_;
        leftCommand.meta.confidence       = 1.0F;
        leftCommand.meta.valid            = true;
        leftCommand.mode                  = model::JointCommandMode::kPosition;
        leftCommand.jointNames            = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint",     "left_wrist_pitch_joint",   "left_wrist_yaw_joint",
        };
        leftCommand.position = {
            0.20 + 0.10 * std::sin(0.7 * t_sec), 0.18 + 0.08 * std::sin(0.5 * t_sec + 0.3),
            0.05 * std::sin(0.4 * t_sec),        0.75 + 0.18 * std::sin(0.9 * t_sec + 0.5),
            0.06 * std::sin(0.8 * t_sec),        0.08 * std::sin(0.6 * t_sec + 0.9),
            0.05 * std::sin(0.5 * t_sec + 1.1),
        };
        frame->jointCommands.push_back(std::move(leftCommand));

        model::PosePrimitive rightPose;
        rightPose.meta.name             = "right_wrist_pose_demo";
        rightPose.meta.entity           = "right_wrist";
        rightPose.meta.bodyGroup        = "right_arm";
        rightPose.meta.frameId          = frameId_;
        rightPose.meta.referenceFrameId = frameId_;
        rightPose.meta.confidence       = 1.0F;
        rightPose.meta.valid            = true;
        rightPose.pose.position.x       = 0.26 + 0.03 * std::cos(0.8 * t_sec);
        rightPose.pose.position.y       = -0.18 + 0.03 * std::sin(0.8 * t_sec);
        rightPose.pose.position.z       = 0.08 + 0.02 * std::sin(0.4 * t_sec + 0.2);
        const double right_yaw          = 0.15 * std::sin(0.8 * t_sec);
        rightPose.pose.orientation.x    = 0.0;
        rightPose.pose.orientation.y    = 0.0;
        rightPose.pose.orientation.z    = std::sin(0.5 * right_yaw);
        rightPose.pose.orientation.w    = std::cos(0.5 * right_yaw);
        rightPose.isRelative            = false;
        rightPose.targetFrameId         = frameId_;
        frame->poses.push_back(std::move(rightPose));

        model::JointStatePrimitive rightSeed;
        rightSeed.meta.name             = "right_arm_seed_state";
        rightSeed.meta.entity           = "right_arm";
        rightSeed.meta.bodyGroup        = "right_arm";
        rightSeed.meta.frameId          = frameId_;
        rightSeed.meta.referenceFrameId = frameId_;
        rightSeed.meta.confidence       = 1.0F;
        rightSeed.meta.valid            = true;
        rightSeed.jointNames            = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint",     "right_wrist_pitch_joint",   "right_wrist_yaw_joint",
        };
        rightSeed.position = {0.2, -0.15, 0.0, 0.45, 0.0, 0.0, 0.0};
        frame->jointStates.push_back(std::move(rightSeed));

        frame->tags["demo"] = "mixed_arm_split";
        populateGroupPipelineIds(leftPipelineId_, rightPipelineId_, frame);
        return true;
    }

    int MixedArmSplitDemoDeviceProvider::suggestedLoopHz() const {
        return loopHz_;
    }

}  // namespace puppet::device
