#include "puppet/device/single_chain_ik/single_chain_ik_sender_device_provider.hpp"

#include <chrono>
#include <cmath>
#include <vector>

namespace puppet::device {

    double SingleChainIkSenderDeviceProvider::squareWaveUnit(double phase) {
        const double p = std::fmod(phase, 1.0);
        if (p < 0.25) {
            return p / 0.25;
        }
        if (p < 0.5) {
            return 1.0;
        }
        if (p < 0.75) {
            return 1.0 - (p - 0.5) / 0.25;
        }
        return 0.0;
    }

    bool SingleChainIkSenderDeviceProvider::initialize(const YAML::Node& configNode, const std::string& frameId,
                                                       const std::string& sourceId, std::string* error) {
        (void)error;
        frameId_  = frameId;
        sourceId_ = sourceId;

        if (configNode["loop_hz"]) {
            loopHz_ = configNode["loop_hz"].as<int>();
        }
        if (configNode["semantic_context"]) {
            semantic_ = configNode["semantic_context"].as<std::string>();
        }
        if (configNode["mode"]) {
            mode_ = configNode["mode"].as<std::string>();
        }
        if (configNode["pipeline_id"]) {
            pipelineId_ = configNode["pipeline_id"].as<std::string>();
        }
        if (configNode["robot_id"]) {
            robotId_ = configNode["robot_id"].as<std::string>();
        }
        if (loopHz_ <= 0) {
            loopHz_ = 50;
        }
        return true;
    }

    bool SingleChainIkSenderDeviceProvider::nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) {
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
        frame->context.pipelineId       = pipelineId_;
        frame->context.robotId          = robotId_;
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

        const double dt    = 1.0 / static_cast<double>(loopHz_);
        const double t_sec = static_cast<double>(sequenceId) * dt;

        const double omega = 0.8;
        const double cx    = 0.26;
        const double cy    = -0.18;
        const double cz    = 0.08;
        const double rx    = 0.03;
        const double yaw   = 0.15 * std::sin(omega * t_sec);

        model::PosePrimitive rightPose;
        rightPose.meta.name             = "right_wrist_pose_demo";
        rightPose.meta.entity           = "right_wrist";
        rightPose.meta.bodyGroup        = model::BodyGroup::kRightArm;
        rightPose.meta.frameId          = frameId_;
        rightPose.meta.referenceFrameId = frameId_;
        rightPose.meta.confidence       = 1.0F;
        rightPose.meta.valid            = true;
        rightPose.pose.position.x       = cx + rx * std::cos(omega * t_sec);
        rightPose.pose.position.y       = cy + rx * std::sin(omega * t_sec);
        rightPose.pose.position.z       = cz;
        rightPose.pose.orientation.x    = 0.0;
        rightPose.pose.orientation.y    = 0.0;
        rightPose.pose.orientation.z    = std::sin(0.5 * yaw);
        rightPose.pose.orientation.w    = std::cos(0.5 * yaw);
        rightPose.isRelative            = false;
        rightPose.targetFrameId         = frameId_;
        frame->poses.push_back(std::move(rightPose));

        const double squarePeriod = 4.0;
        const double phase        = std::fmod(t_sec / squarePeriod, 1.0);
        const double edge         = 0.05;
        double leftX              = 0.24;
        double leftY              = 0.18;
        double leftZ              = 0.10;

        if (phase < 0.25) {
            leftX += edge * (phase / 0.25);
            leftZ += edge;
        } else if (phase < 0.5) {
            leftX += edge;
            leftZ += edge * (1.0 - (phase - 0.25) / 0.25);
        } else if (phase < 0.75) {
            leftX += edge * (1.0 - (phase - 0.5) / 0.25);
        } else {
            leftZ += edge * ((phase - 0.75) / 0.25);
        }
        const double leftYaw = 0.5 * (2.0 * squareWaveUnit(phase) - 1.0);

        model::PosePrimitive leftPose;
        leftPose.meta.name             = "left_wrist_pose_demo";
        leftPose.meta.entity           = "left_wrist";
        leftPose.meta.bodyGroup        = model::BodyGroup::kLeftArm;
        leftPose.meta.frameId          = frameId_;
        leftPose.meta.referenceFrameId = frameId_;
        leftPose.meta.confidence       = 1.0F;
        leftPose.meta.valid            = true;
        leftPose.pose.position.x       = leftX;
        leftPose.pose.position.y       = leftY;
        leftPose.pose.position.z       = leftZ;
        leftPose.pose.orientation.x    = 0.0;
        leftPose.pose.orientation.y    = 0.0;
        leftPose.pose.orientation.z    = std::sin(0.5 * leftYaw);
        leftPose.pose.orientation.w    = std::cos(0.5 * leftYaw);
        leftPose.isRelative            = false;
        leftPose.targetFrameId         = frameId_;
        frame->poses.push_back(std::move(leftPose));

        model::JointStatePrimitive rightSeed;
        rightSeed.meta.name             = "right_arm_seed_state";
        rightSeed.meta.entity           = "right_arm";
        rightSeed.meta.bodyGroup        = model::BodyGroup::kRightArm;
        rightSeed.meta.frameId          = frameId_;
        rightSeed.meta.referenceFrameId = frameId_;
        rightSeed.meta.confidence       = 1.0F;
        rightSeed.meta.valid            = true;

        const std::vector<std::string> rightJointNames = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint",     "right_wrist_pitch_joint",   "right_wrist_yaw_joint",
        };
        const std::vector<double> rightSeedPositions = {0.2, -0.15, 0.0, 0.45, 0.0, 0.0, 0.0};
        for (size_t index = 0; index < rightJointNames.size(); ++index) {
            rightSeed.jointNames.push_back(rightJointNames[index]);
            rightSeed.position.push_back(rightSeedPositions[index]);
        }
        frame->jointStates.push_back(std::move(rightSeed));

        model::JointStatePrimitive leftSeed;
        leftSeed.meta.name             = "left_arm_seed_state";
        leftSeed.meta.entity           = "left_arm";
        leftSeed.meta.bodyGroup        = model::BodyGroup::kLeftArm;
        leftSeed.meta.frameId          = frameId_;
        leftSeed.meta.referenceFrameId = frameId_;
        leftSeed.meta.confidence       = 1.0F;
        leftSeed.meta.valid            = true;

        const std::vector<std::string> leftJointNames = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint",     "left_wrist_pitch_joint",   "left_wrist_yaw_joint",
        };
        const std::vector<double> leftSeedPositions = {0.2, 0.15, 0.0, 0.45, 0.0, 0.0, 0.0};
        for (size_t index = 0; index < leftJointNames.size(); ++index) {
            leftSeed.jointNames.push_back(leftJointNames[index]);
            leftSeed.position.push_back(leftSeedPositions[index]);
        }
        frame->jointStates.push_back(std::move(leftSeed));

        frame->tags["demo"] = "single_chain_ik";
        return true;
    }

    int SingleChainIkSenderDeviceProvider::suggestedLoopHz() const {
        return loopHz_;
    }

}  // namespace puppet::device
