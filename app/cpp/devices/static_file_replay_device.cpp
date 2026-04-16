#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "embosa.hpp"
#include "gmr/retarget/human_frame_io.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/uniform.pb.h"

namespace {

    struct ReplayConfig {
        std::string nodeName  = "puppet_static_file_replay_device";
        std::string topicName = "puppet_demo/primitive_frame";
        std::string sourceId  = "static_file_device";
        std::string humanFrameJson;
        int loopHz        = 30;
        bool loopPlayback = true;
    };

    bool loadConfig(const std::string& path, ReplayConfig* cfg, std::string* error) {
        try {
            YAML::Node root = YAML::LoadFile(path);
            if (root["node_name"])
                cfg->nodeName = root["node_name"].as<std::string>();
            if (root["topic_name"])
                cfg->topicName = root["topic_name"].as<std::string>();
            if (root["source_id"])
                cfg->sourceId = root["source_id"].as<std::string>();
            if (root["human_frame_json"]) {
                cfg->humanFrameJson = root["human_frame_json"].as<std::string>();
            }
            if (root["loop_hz"])
                cfg->loopHz = root["loop_hz"].as<int>();
            if (root["loop_playback"])
                cfg->loopPlayback = root["loop_playback"].as<bool>();
            if (cfg->humanFrameJson.empty()) {
                *error = "human_frame_json is empty";
                return false;
            }
            return true;
        } catch (const std::exception& ex) {
            *error = ex.what();
            return false;
        }
    }

    std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame> toPrimitiveFrame(const gmr::HumanFrame& humanFrame, const ReplayConfig& cfg,
                                                                             uint64_t seq) {
        auto frame = std::make_shared<::puppet::puppet_proto::PrimitiveFrame>();

        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->mutable_header()->mutable_timestamp()->set_sec(sec);
        frame->mutable_header()->mutable_timestamp()->set_nanosec(static_cast<uint32_t>(nsec));
        frame->mutable_header()->set_frame_id("world");

        frame->set_sequence_id(seq);
        frame->mutable_context()->set_source_id(cfg.sourceId);
        frame->mutable_context()->set_source_type(::puppet::puppet_proto::SourceType::SOURCE_TYPE_EXTERNAL);
        frame->mutable_context()->set_semantic_context("human_frame_replay");
        frame->mutable_context()->set_mode("gmr");
        frame->mutable_context()->set_pipeline_id("gmr_pipeline");

        for (const auto& kv : humanFrame) {
            const auto& name = kv.first;
            const auto& body = kv.second;

            auto* pose = frame->add_poses();
            pose->mutable_meta()->set_name(name + "_pose");
            pose->mutable_meta()->set_entity(name);
            pose->mutable_meta()->set_body_group(::puppet::puppet_proto::BodyGroup::BODY_GROUP_WHOLE_BODY);
            pose->mutable_meta()->set_frame_id("world");
            pose->mutable_meta()->set_reference_frame_id("world");
            pose->mutable_meta()->set_confidence(1.0F);
            pose->mutable_meta()->set_valid(true);

            pose->mutable_pose()->mutable_position()->set_x(body.position.x());
            pose->mutable_pose()->mutable_position()->set_y(body.position.y());
            pose->mutable_pose()->mutable_position()->set_z(body.position.z());

            pose->mutable_pose()->mutable_orientation()->set_w(body.orientation.w());
            pose->mutable_pose()->mutable_orientation()->set_x(body.orientation.x());
            pose->mutable_pose()->mutable_orientation()->set_y(body.orientation.y());
            pose->mutable_pose()->mutable_orientation()->set_z(body.orientation.z());

            pose->set_is_relative(false);
            pose->set_target_frame_id("world");
        }

        return frame;
    }

}  // namespace

int main(int argc, char** argv) {
    std::string configPath = "config/device/static_file_replay_device.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    ReplayConfig cfg;
    std::string error;
    if (!loadConfig(configPath, &cfg, &error)) {
        std::cerr << "[static_file_replay_device] load config failed: " << error << std::endl;
        return 1;
    }

    gmr::HumanFrameSequence seq;
    try {
        seq = gmr::loadHumanFrameSequence(cfg.humanFrameJson);
    } catch (const std::exception& ex) {
        std::cerr << "[static_file_replay_device] parse human frame json failed: " << ex.what() << std::endl;
        return 2;
    }

    const int hz       = cfg.loopHz > 0 ? cfg.loopHz : seq.fps;
    const auto sleepDt = std::chrono::milliseconds(1000 / (hz > 0 ? hz : 30));

    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode(cfg.nodeName);
    if (node == nullptr) {
        std::cerr << "[static_file_replay_device] create node failed" << std::endl;
        return 3;
    }

    auto writer = node->CreateWriter<::puppet::puppet_proto::PrimitiveFrame>(cfg.topicName);
    if (writer == nullptr) {
        std::cerr << "[static_file_replay_device] create writer failed" << std::endl;
        return 4;
    }

    size_t frameIdx     = 0;
    uint64_t publishSeq = 0;
    while (galbot::embosa::OK()) {
        const auto msg = toPrimitiveFrame(seq.frames[frameIdx], cfg, publishSeq);
        writer->Publish(msg);

        std::cout << "[static_file_replay_device] publish seq=" << publishSeq << " frame_idx=" << frameIdx << " poses=" << msg->poses_size()
                  << std::endl;

        ++publishSeq;
        ++frameIdx;

        if (frameIdx >= seq.frames.size()) {
            if (!cfg.loopPlayback) {
                break;
            }
            frameIdx = 0;
        }

        std::this_thread::sleep_for(sleepDt);
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
