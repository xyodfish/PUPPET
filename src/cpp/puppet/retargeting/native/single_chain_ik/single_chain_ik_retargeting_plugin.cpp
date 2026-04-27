#include "puppet/retargeting/native/single_chain_ik_retargeting_plugin.hpp"

#include <glog/logging.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

namespace puppet::retargeting {

    namespace single_chain_ik_internal {
        constexpr uint32_t kMaxConsecutiveIkFailuresBeforeSeedReset = 5;

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
            if (bodyGroup == "torso") {
                return model::BodyGroup::kTorso;
            }
            if (bodyGroup == "whole_body") {
                return model::BodyGroup::kWholeBody;
            }
            return model::BodyGroup::kCustom;
        }

        KDL::Frame poseToKdlFrame(const model::Pose& pose) {
            const auto& p = pose.position;
            const auto& q = pose.orientation;
            return KDL::Frame(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w), KDL::Vector(p.x, p.y, p.z));
        }

        bool findSeedFromInput(const model::PrimitiveFrame& input, const std::vector<std::string>& jointNames, KDL::JntArray* seed) {
            if (seed == nullptr) {
                return false;
            }
            if (input.jointStates.empty()) {
                return false;
            }

            std::unordered_map<std::string, double> jointValueMap;
            for (const auto& jointState : input.jointStates) {
                const size_t size = std::min(jointState.jointNames.size(), jointState.position.size());
                for (size_t i = 0; i < size; ++i) {
                    jointValueMap[jointState.jointNames[i]] = jointState.position[i];
                }
            }

            bool has_any = false;
            for (size_t i = 0; i < jointNames.size(); ++i) {
                const auto it = jointValueMap.find(jointNames[i]);
                if (it != jointValueMap.end()) {
                    (*seed)(i) = it->second;
                    has_any    = true;
                }
            }
            return has_any;
        }

        double findNearestEquivalent(double value, double reference, double lower, double upper) {
            const double kTwoPi = 2.0 * KDL::PI;
            double best         = std::min(upper, std::max(lower, value));
            double bestDistance = std::abs(best - reference);
            for (int k = -2; k <= 2; ++k) {
                const double candidate = value + static_cast<double>(k) * kTwoPi;
                if (candidate < lower || candidate > upper) {
                    continue;
                }
                const double distance = std::abs(candidate - reference);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    best         = candidate;
                }
            }
            return best;
        }

    }  // namespace single_chain_ik_internal

    bool SingleChainIkRetargetingPlugin::configure(const runtime::RuntimeConfig& config, std::string* error) {
        enabled_  = config.singleChainIk.enabled;
        chainMap_ = config.singleChainIk.chainMap;
        dtSec_    = config.loopHz > 0 ? (1.0 / static_cast<double>(config.loopHz)) : 0.01;

        if (chainMap_.empty()) {
            if (error != nullptr) {
                *error = "single_chain_ik chains is empty";
            }
            return false;
        }

        chainContextMap_.clear();
        poseStateMap_.clear();

        for (const auto& [bodyGroup, chainCfg] : chainMap_) {
            auto solver = std::make_unique<TRAC_IK::TRAC_IK>(chainCfg.baseLink, chainCfg.tipLink, chainCfg.urdfPath, chainCfg.maxIterations,
                                                             chainCfg.timeoutSec, chainCfg.epsilon, chainCfg.randomRestart, false, false,
                                                             TRAC_IK::Speed);

            ChainContext ctx;
            ctx.config = chainCfg;
            if (!solver->getKDLChain(ctx.chain) || !solver->getKDLLimits(ctx.lowerLimit, ctx.upperLimit)) {
                if (error != nullptr) {
                    *error = "single_chain_ik failed to initialize chain from TRAC-IK for body_group: " + bodyGroup;
                }
                return false;
            }

            const auto jointCount = static_cast<size_t>(ctx.chain.getNrOfJoints());
            if (jointCount != chainCfg.jointNames.size()) {
                if (error != nullptr) {
                    *error = "single_chain_ik joint_names size mismatch for body_group: " + bodyGroup;
                }
                return false;
            }

            ctx.seedJoints = KDL::JntArray(jointCount);
            for (size_t i = 0; i < jointCount; ++i) {
                if (i < chainCfg.defaultPosture.size()) {
                    ctx.seedJoints(i) = chainCfg.defaultPosture[i];
                } else {
                    ctx.seedJoints(i) = 0.5 * (ctx.lowerLimit(i) + ctx.upperLimit(i));
                }
            }

            ctx.lastSolution = KDL::JntArray(jointCount);
            ctx.solver       = std::move(solver);

            chainContextMap_.emplace(bodyGroup, std::move(ctx));

            LOG(INFO) << "single_chain_ik configured body_group=" << bodyGroup << " base_link=" << chainCfg.baseLink
                      << " tip_link=" << chainCfg.tipLink << " joints=" << jointCount;
        }

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    const model::PosePrimitive* SingleChainIkRetargetingPlugin::selectTargetPose(const model::PrimitiveFrame& input,
                                                                                 const std::string& bodyGroup,
                                                                                 const ChainContext& chainCtx) const {
        const model::PosePrimitive* selectedPose = nullptr;
        if (!chainCtx.config.eeEntity.empty()) {
            for (const auto& pose : input.poses) {
                if (pose.meta.entity == chainCtx.config.eeEntity) {
                    selectedPose = &pose;
                    break;
                }
            }
        }
        if (selectedPose == nullptr) {
            const auto targetBodyGroup = single_chain_ik_internal::toBodyGroup(bodyGroup);
            for (const auto& pose : input.poses) {
                if (pose.meta.bodyGroup == targetBodyGroup) {
                    selectedPose = &pose;
                    break;
                }
            }
        }
        if (selectedPose == nullptr && !input.poses.empty()) {
            selectedPose = &input.poses.front();
        }
        return selectedPose;
    }

    KDL::JntArray SingleChainIkRetargetingPlugin::makeIkSeed(const ChainContext& chainCtx) const {
        const size_t jointN = static_cast<size_t>(chainCtx.chain.getNrOfJoints());
        KDL::JntArray seed  = chainCtx.hasLastSolution ? chainCtx.lastSolution : chainCtx.seedJoints;
        for (size_t i = 0; i < jointN; ++i) {
            seed(i) = std::min(chainCtx.upperLimit(i), std::max(chainCtx.lowerLimit(i), seed(i)));
        }
        return seed;
    }

    bool SingleChainIkRetargetingPlugin::solveIkWithPerturbation(const ChainContext& chainCtx, const KDL::JntArray& seed,
                                                                 const KDL::Frame& targetPose, KDL::JntArray* solution) const {
        if (solution == nullptr) {
            return false;
        }
        bool solved = chainCtx.solver->CartToJnt(seed, targetPose, *solution) >= 0;
        if (solved) {
            return true;
        }

        std::mt19937 generator(std::random_device{}());
        std::uniform_real_distribution<double> posDis(-chainCtx.config.perturbationPositionRange,
                                                      chainCtx.config.perturbationPositionRange);
        std::uniform_real_distribution<double> angleDis(-chainCtx.config.perturbationAngleDegRange * KDL::deg2rad,
                                                        chainCtx.config.perturbationAngleDegRange * KDL::deg2rad);
        for (int32_t i = 0; i < chainCtx.config.perturbationMaxAttempts; ++i) {
            KDL::Frame perturbed = targetPose;
            perturbed.p.x(targetPose.p.x() + posDis(generator));
            perturbed.p.y(targetPose.p.y() + posDis(generator));
            perturbed.p.z(targetPose.p.z() + posDis(generator));
            const KDL::Rotation rotOffset = KDL::Rotation::RPY(angleDis(generator), angleDis(generator), angleDis(generator));
            perturbed.M                   = targetPose.M * rotOffset;

            if (chainCtx.solver->CartToJnt(seed, perturbed, *solution) >= 0) {
                return true;
            }
        }
        return false;
    }

    void SingleChainIkRetargetingPlugin::handleSolveFailure(ChainContext* chainCtx) const {
        if (chainCtx == nullptr) {
            return;
        }
        chainCtx->consecutiveFailureCount++;
        if (chainCtx->consecutiveFailureCount >= single_chain_ik_internal::kMaxConsecutiveIkFailuresBeforeSeedReset) {
            chainCtx->hasLastSolution         = false;
            chainCtx->consecutiveFailureCount = 0;
        }
    }

    model::JointCommandIntent SingleChainIkRetargetingPlugin::buildJointCommandIntent(const std::string& bodyGroup, ChainContext* chainCtx,
                                                                                      const KDL::JntArray& solutionBeforeClamp) const {
        const size_t jointN                  = static_cast<size_t>(chainCtx->chain.getNrOfJoints());
        const bool hadLastSolution           = chainCtx->hasLastSolution;
        const KDL::JntArray previousSolution = chainCtx->lastSolution;
        KDL::JntArray finalSolution          = solutionBeforeClamp;

        model::JointCommandIntent jointIntent;
        jointIntent.bodyGroup  = single_chain_ik_internal::toBodyGroup(bodyGroup);
        jointIntent.mode       = model::JointCommandIntent::Mode::kPosition;
        jointIntent.jointNames = chainCtx->config.jointNames;
        jointIntent.position.resize(jointN, 0.0);
        jointIntent.weight = 1.0;

        for (size_t i = 0; i < jointN; ++i) {
            double value = finalSolution(i);
            if (i < chainCtx->config.minPositionLimit.size() && i < chainCtx->config.maxPositionLimit.size()) {
                value = std::min(chainCtx->config.maxPositionLimit[i], std::max(chainCtx->config.minPositionLimit[i], value));
            }
            if (hadLastSolution) {
                value = single_chain_ik_internal::findNearestEquivalent(value, previousSolution(i), chainCtx->lowerLimit(i),
                                                                        chainCtx->upperLimit(i));
            }
            jointIntent.position[i] = value;
            finalSolution(i)        = value;
        }

        chainCtx->lastSolution    = finalSolution;
        chainCtx->hasLastSolution = true;
        return jointIntent;
    }

    bool SingleChainIkRetargetingPlugin::process(const model::PrimitiveFrame& input, const std::string& bodyGroup,
                                                 model::GroupControlIntent* output, std::string* error) {
        if (output == nullptr) {
            if (error != nullptr) {
                *error = "output is null";
            }
            return false;
        }
        if (!enabled_) {
            if (error != nullptr) {
                *error = "single_chain_ik plugin is disabled";
            }
            return false;
        }

        auto chainIt = chainContextMap_.find(bodyGroup);
        if (chainIt == chainContextMap_.end()) {
            if (error != nullptr) {
                *error = "single_chain_ik chain config not found for body_group: " + bodyGroup;
            }
            return false;
        }
        ChainContext& chainCtx = chainIt->second;

        const model::PrimitiveFrame runtimeInput = preprocessToPoseFrame(input, bodyGroup);
        const model::PosePrimitive* selectedPose = selectTargetPose(runtimeInput, bodyGroup, chainCtx);
        if (selectedPose == nullptr) {
            if (error != nullptr) {
                *error = "single_chain_ik input poses is empty";
            }
            return false;
        }

        const KDL::JntArray seed    = makeIkSeed(chainCtx);
        const KDL::Frame targetPose = single_chain_ik_internal::poseToKdlFrame(selectedPose->pose);
        KDL::JntArray ikSolution(static_cast<size_t>(chainCtx.chain.getNrOfJoints()));
        if (!solveIkWithPerturbation(chainCtx, seed, targetPose, &ikSolution)) {
            handleSolveFailure(&chainCtx);
            if (error != nullptr) {
                *error = "single_chain_ik trac_ik solve failed for body_group: " + bodyGroup;
            }
            return false;
        }
        chainCtx.consecutiveFailureCount = 0;

        model::JointCommandIntent jointIntent = buildJointCommandIntent(bodyGroup, &chainCtx, ikSolution);
        output->bodyGroup                     = jointIntent.bodyGroup;
        output->ownerSourceId                 = input.context.sourceId;
        output->mode                          = "single_chain_ik";
        output->enabled                       = true;
        output->jointCommandIntents.push_back(std::move(jointIntent));
        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

    model::PrimitiveFrame SingleChainIkRetargetingPlugin::preprocessToPoseFrame(const model::PrimitiveFrame& input,
                                                                                const std::string& bodyGroup) const {
        model::PrimitiveFrame output = input;
        output.poses.clear();

        if (input.context.mode == "twist") {
            for (const auto& twistPrimitive : input.twists) {
                if (twistPrimitive.meta.entity.empty()) {
                    continue;
                }
                const std::string stateKey = makeStateKey(bodyGroup, twistPrimitive.meta.entity);
                auto stateIt               = poseStateMap_.find(stateKey);
                if (stateIt == poseStateMap_.end()) {
                    if (seedPoseStateFromRobotJointState(input, bodyGroup, twistPrimitive.meta.entity)) {
                        stateIt = poseStateMap_.find(stateKey);
                    }
                }
                if (stateIt == poseStateMap_.end()) {
                    static uint64_t noSeedWarnCount = 0;
                    ++noSeedWarnCount;
                    if ((noSeedWarnCount % 100ULL) == 1ULL) {
                        LOG(WARNING) << "single_chain_ik skip twist integration due to missing pose seed, body_group=" << bodyGroup
                                     << " entity=" << twistPrimitive.meta.entity;
                    }
                    continue;
                }
                auto& poseState = stateIt->second;
                poseState.position.x += twistPrimitive.twist.linear.x * dtSec_;
                poseState.position.y += twistPrimitive.twist.linear.y * dtSec_;
                poseState.position.z += twistPrimitive.twist.linear.z * dtSec_;

                model::PosePrimitive posePrimitive;
                posePrimitive.meta                  = twistPrimitive.meta;
                posePrimitive.meta.frameId          = twistPrimitive.bodyFrameId;
                posePrimitive.meta.referenceFrameId = twistPrimitive.referenceFrameId;
                posePrimitive.pose                  = poseState;
                posePrimitive.isRelative            = false;
                posePrimitive.targetFrameId         = twistPrimitive.referenceFrameId;
                output.poses.push_back(std::move(posePrimitive));
            }
        }

        for (const auto& posePrimitive : input.poses) {
            if (!posePrimitive.meta.entity.empty()) {
                poseStateMap_[makeStateKey(bodyGroup, posePrimitive.meta.entity)] = posePrimitive.pose;
            }
            output.poses.push_back(posePrimitive);
        }

        return output;
    }

    std::string SingleChainIkRetargetingPlugin::makeStateKey(const std::string& bodyGroup, const std::string& entity) {
        return bodyGroup + "::" + entity;
    }

    bool SingleChainIkRetargetingPlugin::seedPoseStateFromRobotJointState(const model::PrimitiveFrame& input, const std::string& bodyGroup,
                                                                          const std::string& entity) const {
        const auto chainIt = chainContextMap_.find(bodyGroup);
        if (chainIt == chainContextMap_.end()) {
            return false;
        }
        const auto& chainCtx = chainIt->second;
        KDL::JntArray joints = chainCtx.seedJoints;
        if (!single_chain_ik_internal::findSeedFromInput(input, chainCtx.config.jointNames, &joints)) {
            return false;
        }

        KDL::ChainFkSolverPos_recursive fkSolver(chainCtx.chain);
        KDL::Frame eeFrame;
        if (fkSolver.JntToCart(joints, eeFrame) < 0) {
            return false;
        }

        model::Pose pose;
        pose.position.x = eeFrame.p.x();
        pose.position.y = eeFrame.p.y();
        pose.position.z = eeFrame.p.z();
        eeFrame.M.GetQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        poseStateMap_[makeStateKey(bodyGroup, entity)] = pose;
        return true;
    }

}  // namespace puppet::retargeting
