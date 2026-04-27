#pragma once

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "puppet/retargeting/core/retargeting_plugin.hpp"

namespace puppet::retargeting {

    class SingleChainIkRetargetingPlugin final : public RetargetingPlugin {
       public:
        std::string name() const override { return "single_chain_ik"; }
        bool configure(const runtime::RuntimeConfig& config, std::string* error) override;
        bool process(const model::PrimitiveFrame& input, const std::string& bodyGroup, model::GroupControlIntent* output,
                     std::string* error) override;

       private:
        struct ChainContext;

        const model::PosePrimitive* selectTargetPose(const model::PrimitiveFrame& input, const std::string& bodyGroup,
                                                     const ChainContext& chainCtx) const;
        KDL::JntArray makeIkSeed(const ChainContext& chainCtx) const;
        bool solveIkWithPerturbation(const ChainContext& chainCtx, const KDL::JntArray& seed, const KDL::Frame& targetPose,
                                     KDL::JntArray* solution) const;
        void handleSolveFailure(ChainContext* chainCtx) const;
        model::JointCommandIntent buildJointCommandIntent(const std::string& bodyGroup, ChainContext* chainCtx,
                                                          const KDL::JntArray& solutionBeforeClamp) const;

        model::PrimitiveFrame preprocessToPoseFrame(const model::PrimitiveFrame& input, const std::string& bodyGroup) const;
        bool seedPoseStateFromRobotJointState(const model::PrimitiveFrame& input, const std::string& bodyGroup,
                                              const std::string& entity) const;
        static std::string makeStateKey(const std::string& bodyGroup, const std::string& entity);

        struct ChainContext {
            runtime::SingleChainIkChainConfig config;
            std::unique_ptr<TRAC_IK::TRAC_IK> solver;
            KDL::Chain chain;
            KDL::JntArray lowerLimit;
            KDL::JntArray upperLimit;
            KDL::JntArray seedJoints;
            KDL::JntArray lastSolution;
            bool hasLastSolution             = false;
            uint32_t consecutiveFailureCount = 0;
        };

        std::unordered_map<std::string, ChainContext> chainContextMap_;
        bool enabled_ = false;
        double dtSec_ = 0.01;
        std::unordered_map<std::string, runtime::SingleChainIkChainConfig> chainMap_;
        mutable std::unordered_map<std::string, model::Pose> poseStateMap_;
    };

}  // namespace puppet::retargeting
