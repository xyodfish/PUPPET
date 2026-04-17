#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "embosa.hpp"
#include "puppet/control_intent.pb.h"
#include "puppet/primitive/primitive_types.hpp"
#include "puppet/primitive_frame.pb.h"
#include "puppet/runtime/teleop_runtime.hpp"
#include "puppet/transport/proto_copy.hpp"

namespace puppet::runtime::embosa_main_internal {

    struct EmbosaRuntimeConfig {
        std::string embosaNodeName           = "puppet_teleop_runtime";
        std::string inputTopicName           = "puppet_demo/primitive_frame";
        std::string outputControlIntentTopic = "puppet_demo/control_intent";
        int idleSleepMs                      = 2;
    };

    bool loadEmbosaRuntimeConfig(const std::string& runtimeYaml, EmbosaRuntimeConfig* cfg, std::string* error) {
        try {
            YAML::Node root = YAML::LoadFile(runtimeYaml);
            if (root["embosa_runtime"]) {
                auto node = root["embosa_runtime"];
                if (node["node_name"]) {
                    cfg->embosaNodeName = node["node_name"].as<std::string>();
                }
                if (node["input_topic_name"]) {
                    cfg->inputTopicName = node["input_topic_name"].as<std::string>();
                } else if (node["topic_name"]) {
                    cfg->inputTopicName = node["topic_name"].as<std::string>();
                }
                if (node["output_control_intent_topic"]) {
                    cfg->outputControlIntentTopic = node["output_control_intent_topic"].as<std::string>();
                }
                if (node["idle_sleep_ms"]) {
                    cfg->idleSleepMs = node["idle_sleep_ms"].as<int>();
                }
            }
            return true;
        } catch (const std::exception& ex) {
            *error = ex.what();
            return false;
        }
    }

    ::puppet::puppet_proto::ControlIntent toProto(const puppet::model::ControlIntent& src) {
        ::puppet::puppet_proto::ControlIntent dst;
        dst.set_sequence_id(src.sequenceId);
        dst.mutable_context()->set_source_id(src.context.sourceId);
        dst.mutable_context()->set_mode(src.context.mode);
        dst.mutable_context()->set_semantic_context(src.context.semanticContext);
        dst.mutable_context()->set_pipeline_id(src.context.pipelineId);

        for (const auto& group : src.groupIntents) {
            auto* groupPb = dst.add_group_intents();
            groupPb->set_owner_source_id(group.ownerSourceId);
            groupPb->set_mode(group.mode);
            groupPb->set_priority(group.priority);
            groupPb->set_backend_hint(group.backendHint);
            groupPb->set_enabled(group.enabled);

            for (const auto& jointIntent : group.jointCommandIntents) {
                auto* jointPb = groupPb->add_joint_command_intents();
                for (const auto& n : jointIntent.jointNames) {
                    jointPb->add_joint_names(n);
                }
                for (double v : jointIntent.position) {
                    jointPb->add_position(v);
                }
                for (double v : jointIntent.velocity) {
                    jointPb->add_velocity(v);
                }
                for (double v : jointIntent.effort) {
                    jointPb->add_effort(v);
                }
                for (double v : jointIntent.stiffness) {
                    jointPb->add_stiffness(v);
                }
                for (double v : jointIntent.damping) {
                    jointPb->add_damping(v);
                }
                jointPb->set_weight(jointIntent.weight);
            }
        }
        return dst;
    }

}  // namespace puppet::runtime::embosa_main_internal

int main(int argc, char** argv) {
    using namespace puppet::runtime::embosa_main_internal;

    std::string runtimeConfig = "config/runtime/teleop_runtime_gmr.yaml";
    if (argc > 1) {
        runtimeConfig = argv[1];
    }

    puppet::runtime::TeleopRuntime runtime;
    std::string error;
    if (!runtime.init(runtimeConfig, &error)) {
        std::cerr << "[teleop_runtime_embosa] init failed: " << error << std::endl;
        return 1;
    }

    EmbosaRuntimeConfig embosaCfg;
    if (!loadEmbosaRuntimeConfig(runtimeConfig, &embosaCfg, &error)) {
        std::cerr << "[teleop_runtime_embosa] load embosa config failed: " << error << std::endl;
        return 2;
    }

    std::mutex queueMutex;
    std::deque<puppet::model::PrimitiveFrame> queue;

    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode(embosaCfg.embosaNodeName);
    if (node == nullptr) {
        std::cerr << "[teleop_runtime_embosa] create node failed" << std::endl;
        return 3;
    }

    auto onMsg = [&](const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg, const void*) {
        if (msg == nullptr) {
            return;
        }
        puppet::model::PrimitiveFrame frame;
        if (!puppet::transport::copyFromProto(*msg, &frame)) {
            return;
        }
        std::lock_guard<std::mutex> lk(queueMutex);
        queue.push_back(frame);
        if (queue.size() > 200) {
            queue.pop_front();
        }

        std::cout << "Received frame " << frame.header.frameId << std::endl;
    };

    std::shared_ptr<galbot::embosa::SerializationReader<::puppet::puppet_proto::PrimitiveFrame>> reader =
        node->CreateReader<::puppet::puppet_proto::PrimitiveFrame>(embosaCfg.inputTopicName, onMsg);
    if (reader == nullptr) {
        std::cerr << "[teleop_runtime_embosa] create reader failed" << std::endl;
        return 4;
    }

    std::shared_ptr<galbot::embosa::SerializationWriter<::puppet::puppet_proto::ControlIntent>> controlIntentWriter =
        node->CreateWriter<::puppet::puppet_proto::ControlIntent>(embosaCfg.outputControlIntentTopic);
    if (controlIntentWriter == nullptr) {
        std::cerr << "[teleop_runtime_embosa] create control intent writer failed" << std::endl;
        return 5;
    }

    while (galbot::embosa::OK()) {
        puppet::model::PrimitiveFrame frame;
        bool hasFrame = false;
        {
            std::lock_guard<std::mutex> lk(queueMutex);
            if (!queue.empty()) {
                frame = std::move(queue.front());
                queue.pop_front();
                hasFrame = true;
            }
        }

        if (!hasFrame) {
            // std::cout << "Waiting for frame..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(embosaCfg.idleSleepMs));
            continue;
        }

        runtime.sourceManager()->ingestFrame(frame);
        if (!runtime.runOnce(&error)) {
            std::cerr << "[teleop_runtime_embosa] runOnce failed: " << error << std::endl;
            continue;
        }

        auto controlIntentPb = std::make_shared<::puppet::puppet_proto::ControlIntent>(toProto(runtime.lastControlIntent()));
        controlIntentWriter->Publish(controlIntentPb);
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
