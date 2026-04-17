#pragma once

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <trac_ik/trac_ik.hpp>

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
        struct ChainContext {
            runtime::SingleChainIkChainConfig config;
            std::unique_ptr<TRAC_IK::TRAC_IK> solver;
            KDL::Chain chain;
            KDL::JntArray lowerLimit;
            KDL::JntArray upperLimit;
            KDL::JntArray seedJoints;
            KDL::JntArray lastSolution;
            bool hasLastSolution = false;
        };

        std::unordered_map<std::string, ChainContext> chainContextMap_;
        bool enabled_ = false;
        std::unordered_map<std::string, runtime::SingleChainIkChainConfig> chainMap_;
    };

}  // namespace puppet::retargeting
