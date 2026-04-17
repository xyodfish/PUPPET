#include "puppet/retargeting/native/single_chain_ik_retargeting_plugin.hpp"

#include <glog/logging.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>

#include <algorithm>
#include <random>
#include <utility>

namespace puppet::retargeting {

    namespace single_chain_ik_internal {

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

    }  // namespace single_chain_ik_internal

    bool SingleChainIkRetargetingPlugin::configure(const runtime::RuntimeConfig& config, std::string* error) {
        enabled_  = config.singleChainIk.enabled;
        chainMap_ = config.singleChainIk.chainMap;

        if (chainMap_.empty()) {
            if (error != nullptr) {
                *error = "single_chain_ik chains is empty";
            }
            return false;
        }

        chainContextMap_.clear();

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

        const auto chainIt = chainContextMap_.find(bodyGroup);
        if (chainIt == chainContextMap_.end()) {
            if (error != nullptr) {
                *error = "single_chain_ik chain config not found for body_group: " + bodyGroup;
            }
            return false;
        }
        auto& chainCtx    = chainIt->second;
        const auto& chain = chainCtx.config;
        const auto jointN = static_cast<size_t>(chainCtx.chain.getNrOfJoints());

        const model::PosePrimitive* selectedPose = nullptr;
        if (!chain.eeEntity.empty()) {
            for (const auto& pose : input.poses) {
                if (pose.meta.entity == chain.eeEntity) {
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

        if (selectedPose == nullptr) {
            if (error != nullptr) {
                *error = "single_chain_ik input poses is empty";
            }
            return false;
        }

        KDL::JntArray seed = chainCtx.seedJoints;
        if (chainCtx.hasLastSolution) {
            seed = chainCtx.lastSolution;
        }
        const bool hasInputSeed = single_chain_ik_internal::findSeedFromInput(input, chain.jointNames, &seed);
        if (hasInputSeed) {
            for (size_t i = 0; i < jointN; ++i) {
                seed(i) = std::min(chainCtx.upperLimit(i), std::max(chainCtx.lowerLimit(i), seed(i)));
            }
        }

        const KDL::Frame targetPose = single_chain_ik_internal::poseToKdlFrame(selectedPose->pose);
        KDL::JntArray solution(jointN);

        bool solved = chainCtx.solver->CartToJnt(seed, targetPose, solution) >= 0;
        if (!solved) {
            std::mt19937 generator(std::random_device{}());
            std::uniform_real_distribution<double> posDis(-chain.perturbationPositionRange, chain.perturbationPositionRange);
            std::uniform_real_distribution<double> angleDis(-chain.perturbationAngleDegRange * KDL::deg2rad,
                                                            chain.perturbationAngleDegRange * KDL::deg2rad);

            for (int32_t i = 0; i < chain.perturbationMaxAttempts; ++i) {
                KDL::Frame perturbed = targetPose;
                perturbed.p.x(targetPose.p.x() + posDis(generator));
                perturbed.p.y(targetPose.p.y() + posDis(generator));
                perturbed.p.z(targetPose.p.z() + posDis(generator));

                const KDL::Rotation rotOffset = KDL::Rotation::RPY(angleDis(generator), angleDis(generator), angleDis(generator));
                perturbed.M                   = targetPose.M * rotOffset;

                if (chainCtx.solver->CartToJnt(seed, perturbed, solution) >= 0) {
                    solved = true;
                    break;
                }
            }
        }

        if (!solved) {
            if (error != nullptr) {
                *error = "single_chain_ik trac_ik solve failed for body_group: " + bodyGroup;
            }
            return false;
        }

        chainCtx.lastSolution    = solution;
        chainCtx.hasLastSolution = true;

        model::JointCommandIntent jointIntent;
        jointIntent.bodyGroup  = single_chain_ik_internal::toBodyGroup(bodyGroup);
        jointIntent.mode       = model::JointCommandIntent::Mode::kPosition;
        jointIntent.jointNames = chain.jointNames;
        jointIntent.position.resize(jointN, 0.0);
        jointIntent.weight = 1.0;

        for (size_t i = 0; i < jointN; ++i) {
            double value = solution(i);
            if (i < chain.minPositionLimit.size() && i < chain.maxPositionLimit.size()) {
                value = std::min(chain.maxPositionLimit[i], std::max(chain.minPositionLimit[i], value));
            }
            jointIntent.position[i] = value;
        }

        output->bodyGroup     = jointIntent.bodyGroup;
        output->ownerSourceId = input.context.sourceId;
        output->mode          = "single_chain_ik";
        output->enabled       = true;
        output->jointCommandIntents.push_back(std::move(jointIntent));

        if (error != nullptr) {
            error->clear();
        }
        return true;
    }

}  // namespace puppet::retargeting
