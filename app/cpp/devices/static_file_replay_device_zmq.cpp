#include <cerrno>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <zmq.h>

#include "gmr/retarget/human_frame_io.h"
#include "puppet/primitive_frame.pb.h"
#include "puppet/uniform.pb.h"

namespace static_file_replay_device_zmq_detail {

    struct ReplayConfig {
        std::string topicName      = "puppet_demo/primitive_frame";
        std::string outputEndpoint = "tcp://127.0.0.1:16661";
        std::string sourceId       = "demo_sender_single_chain_ik";
        std::string humanFrameJson;
        int loopHz        = 30;
        bool loopPlayback = true;
    };

    bool loadConfig(const std::string& path, ReplayConfig* cfg, std::string* error) {
        try {
            YAML::Node root = YAML::LoadFile(path);
            if (root["topic_name"])
                cfg->topicName = root["topic_name"].as<std::string>();
            if (root["output_endpoint"]) {
                cfg->outputEndpoint = root["output_endpoint"].as<std::string>();
            }
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

    ::puppet::puppet_proto::PrimitiveFrame toPrimitiveFrame(const gmr::HumanFrame& humanFrame, const ReplayConfig& cfg, uint64_t seq) {
        ::puppet::puppet_proto::PrimitiveFrame frame;

        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame.mutable_header()->mutable_timestamp()->set_sec(sec);
        frame.mutable_header()->mutable_timestamp()->set_nanosec(static_cast<uint32_t>(nsec));
        frame.mutable_header()->set_frame_id("world");

        frame.set_sequence_id(seq);
        frame.mutable_context()->set_source_id(cfg.sourceId);
        frame.mutable_context()->set_source_type(::puppet::puppet_proto::SourceType::SOURCE_TYPE_EXTERNAL);
        frame.mutable_context()->set_semantic_context("human_frame_replay");
        frame.mutable_context()->set_mode("gmr");
        frame.mutable_context()->set_pipeline_id("gmr_pipeline");

        for (const auto& kv : humanFrame) {
            const auto& name = kv.first;
            const auto& body = kv.second;

            auto* pose = frame.add_poses();
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

    bool publishFrame(void* publisher, const std::string& topicName, const ::puppet::puppet_proto::PrimitiveFrame& frame,
                      std::string* error) {
        std::string payload(frame.ByteSizeLong(), '\0');
        if (!frame.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
            *error = "serialize primitive frame failed";
            return false;
        }

        const int topicRc = zmq_send(publisher, topicName.data(), topicName.size(), ZMQ_SNDMORE);
        if (topicRc < 0) {
            *error = std::string("zmq send topic failed: ") + zmq_strerror(errno);
            return false;
        }
        const int payloadRc = zmq_send(publisher, payload.data(), payload.size(), 0);
        if (payloadRc < 0) {
            *error = std::string("zmq send payload failed: ") + zmq_strerror(errno);
            return false;
        }
        return true;
    }

}  // namespace static_file_replay_device_zmq_detail

int main(int argc, char** argv) {
    std::string configPath = "config/device/static_file_replay_device.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    static_file_replay_device_zmq_detail::ReplayConfig cfg;
    std::string error;
    if (!static_file_replay_device_zmq_detail::loadConfig(configPath, &cfg, &error)) {
        std::cerr << "[static_file_replay_device_zmq] load config failed: " << error << std::endl;
        return 1;
    }

    gmr::HumanFrameSequence seq;
    try {
        seq = gmr::loadHumanFrameSequence(cfg.humanFrameJson);
    } catch (const std::exception& ex) {
        std::cerr << "[static_file_replay_device_zmq] parse human frame json failed: " << ex.what() << std::endl;
        return 2;
    }

    const int hz       = cfg.loopHz > 0 ? cfg.loopHz : seq.fps;
    const auto sleepDt = std::chrono::milliseconds(1000 / (hz > 0 ? hz : 30));

    void* zmqContext = zmq_ctx_new();
    if (zmqContext == nullptr) {
        std::cerr << "[static_file_replay_device_zmq] create zmq context failed: " << zmq_strerror(errno) << std::endl;
        return 3;
    }
    void* publisher = zmq_socket(zmqContext, ZMQ_PUB);
    if (publisher == nullptr) {
        std::cerr << "[static_file_replay_device_zmq] create publisher failed: " << zmq_strerror(errno) << std::endl;
        zmq_ctx_term(zmqContext);
        return 4;
    }
    if (zmq_bind(publisher, cfg.outputEndpoint.c_str()) != 0) {
        std::cerr << "[static_file_replay_device_zmq] bind publisher failed endpoint=" << cfg.outputEndpoint
                  << " error=" << zmq_strerror(errno) << std::endl;
        zmq_close(publisher);
        zmq_ctx_term(zmqContext);
        return 5;
    }

    size_t frameIdx     = 0;
    uint64_t publishSeq = 0;
    while (true) {
        const auto msg = static_file_replay_device_zmq_detail::toPrimitiveFrame(seq.frames[frameIdx], cfg, publishSeq);
        if (!static_file_replay_device_zmq_detail::publishFrame(publisher, cfg.topicName, msg, &error)) {
            std::cerr << "[static_file_replay_device_zmq] publish failed: " << error << std::endl;
            break;
        }

        std::cout << "[static_file_replay_device_zmq] publish seq=" << publishSeq << " frame_idx=" << frameIdx
                  << " poses=" << msg.poses_size() << std::endl;

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

    zmq_close(publisher);
    zmq_ctx_term(zmqContext);
    return 0;
}
