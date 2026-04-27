#include "puppet/device/static_file_replay/static_file_replay_device_provider.hpp"

#include <chrono>
#include <exception>

namespace puppet::device {

    bool StaticFileReplayDeviceProvider::initialize(const YAML::Node& configNode, const std::string& frameId, const std::string& sourceId,
                                                    std::string* error) {
        frameId_  = frameId;
        sourceId_ = sourceId;

        std::string humanFrameJson;
        if (configNode["human_frame_json"]) {
            humanFrameJson = configNode["human_frame_json"].as<std::string>();
        }
        if (humanFrameJson.empty()) {
            *error = "device.human_frame_json is empty";
            return false;
        }

        if (configNode["loop_playback"]) {
            loopPlayback_ = configNode["loop_playback"].as<bool>();
        }
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

        try {
            sequence_ = gmr::loadHumanFrameSequence(humanFrameJson);
        } catch (const std::exception& ex) {
            *error = ex.what();
            return false;
        }
        if (sequence_.frames.empty()) {
            *error = "human frame sequence is empty";
            return false;
        }

        if (loopHz_ <= 0) {
            loopHz_ = sequence_.fps > 0 ? sequence_.fps : 30;
        }

        frameIndex_ = 0;
        return true;
    }

    bool StaticFileReplayDeviceProvider::nextFrame(uint64_t sequenceId, model::PrimitiveFrame* frame, std::string* error) {
        if (frameIndex_ >= sequence_.frames.size()) {
            if (!loopPlayback_) {
                *error = "replay finished";
                return false;
            }
            frameIndex_ = 0;
        }
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

        const auto& humanFrame = sequence_.frames[frameIndex_];
        frame->poses.reserve(humanFrame.size());
        for (const auto& kv : humanFrame) {
            const auto& name = kv.first;
            const auto& body = kv.second;

            model::PosePrimitive pose;
            pose.meta.name             = name + "_pose";
            pose.meta.entity           = name;
            pose.meta.bodyGroup        = model::BodyGroup::kWholeBody;
            pose.meta.frameId          = frameId_;
            pose.meta.referenceFrameId = frameId_;
            pose.meta.confidence       = 1.0F;
            pose.meta.valid            = true;

            pose.pose.position.x = body.position.x();
            pose.pose.position.y = body.position.y();
            pose.pose.position.z = body.position.z();

            pose.pose.orientation.w = body.orientation.w();
            pose.pose.orientation.x = body.orientation.x();
            pose.pose.orientation.y = body.orientation.y();
            pose.pose.orientation.z = body.orientation.z();

            pose.isRelative    = false;
            pose.targetFrameId = frameId_;
            frame->poses.push_back(std::move(pose));
        }
        ++frameIndex_;
        return true;
    }

    int StaticFileReplayDeviceProvider::suggestedLoopHz() const {
        return loopHz_;
    }

}  // namespace puppet::device
