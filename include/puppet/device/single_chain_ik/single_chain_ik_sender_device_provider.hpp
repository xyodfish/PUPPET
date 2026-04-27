#pragma once

#include <string>

#include "puppet/device/device_provider.hpp"

namespace puppet::device {

    class SingleChainIkSenderDeviceProvider : public IDeviceProvider {
       public:
        bool initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId, std::string* error) override;

        bool nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) override;

        int suggestedLoopHz() const override;

       private:
        static double squareWaveUnit(double phase);

        std::string frameId_    = "torso_link";
        std::string sourceId_   = "demo_sender_single_chain_ik";
        std::string semantic_   = "single_chain_ik_demo";
        std::string mode_       = "cart_pose_to_joint";
        std::string pipelineId_ = "single_chain_ik_pipeline";
        std::string robotId_    = "unitree_g1";
        int loopHz_             = 50;
    };

}  // namespace puppet::device
