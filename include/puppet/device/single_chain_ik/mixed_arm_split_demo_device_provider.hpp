#pragma once

#include <string>

#include "puppet/device/device_provider.hpp"

namespace puppet::device {

    class MixedArmSplitDemoDeviceProvider : public IDeviceProvider {
       public:
        bool initialize(const DeviceServiceConfig& config, std::string& error) override;

        bool nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string& error) override;

        int suggestedLoopHz() const override;

       private:
        std::string frameId_         = "torso_link";
        std::string sourceId_        = "demo_sender_mixed_arm_split";
        std::string semantic_        = "mixed_arm_split_demo";
        std::string mode_            = "mixed_arm_split";
        std::string leftPipelineId_  = "direct_pass_pipeline";
        std::string rightPipelineId_ = "single_chain_ik_pipeline";
        std::string robotId_         = "unitree_g1";
        int loopHz_                  = 50;
    };

}  // namespace puppet::device
