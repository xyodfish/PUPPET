#pragma once

#include <string>

#include "gmr/retarget/human_frame_io.h"
#include "puppet/device/device_provider.hpp"

namespace puppet::device {

    class StaticFileReplayDeviceProvider : public IDeviceProvider {
       public:
        bool initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId, std::string* error) override;

        bool nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) override;

        int suggestedLoopHz() const override;

       private:
        std::string frameId_    = "world";
        std::string sourceId_   = "static_file_device";
        std::string semantic_   = "human_frame_replay";
        std::string mode_       = "gmr";
        std::string pipelineId_ = "gmr_pipeline";
        bool loopPlayback_      = true;
        int loopHz_             = 30;
        size_t frameIndex_      = 0;
        gmr::HumanFrameSequence sequence_;
    };

}  // namespace puppet::device
