#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include "embosa.hpp"
#include "puppet/primitive_frame.pb.h"
#include "puppet/uniform.pb.h"

namespace {

    using PrimitiveFramePb = ::puppet::puppet_proto::PrimitiveFrame;
    using SourceTypePb     = ::puppet::puppet_proto::SourceType;
    using BodyGroupPb      = ::puppet::puppet_proto::BodyGroup;

    std::shared_ptr<PrimitiveFramePb> BuildFrame(uint64_t sequenceId, double tSec) {
        auto frame = std::make_shared<PrimitiveFramePb>();

        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->mutable_header()->mutable_timestamp()->set_sec(sec);
        frame->mutable_header()->mutable_timestamp()->set_nanosec(static_cast<uint32_t>(nsec));
        frame->mutable_header()->set_frame_id("world");

        frame->set_sequence_id(sequenceId);
        frame->mutable_context()->set_source_id("demo_sender_node");
        frame->mutable_context()->set_source_type(SourceTypePb::SOURCE_TYPE_EXTERNAL);
        frame->mutable_context()->set_semantic_context("teleop");
        frame->mutable_context()->set_mode("direct_pose");
        frame->mutable_context()->set_robot_id("puppet_demo_robot");
        frame->mutable_context()->set_pipeline_id("demo_pipeline");

        auto* pose = frame->add_poses();
        pose->mutable_meta()->set_name("left_hand_pose");
        pose->mutable_meta()->set_entity("left_hand");
        pose->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_LEFT_ARM);
        pose->mutable_meta()->set_frame_id("world");
        pose->mutable_meta()->set_reference_frame_id("world");
        pose->mutable_meta()->set_confidence(1.0F);
        pose->mutable_meta()->set_valid(true);
        pose->mutable_pose()->mutable_position()->set_x(0.4 + 0.1 * std::sin(tSec));
        pose->mutable_pose()->mutable_position()->set_y(0.2);
        pose->mutable_pose()->mutable_position()->set_z(1.0);
        pose->mutable_pose()->mutable_orientation()->set_w(1.0);
        pose->set_is_relative(false);
        pose->set_target_frame_id("world");

        auto* scalar = frame->add_scalars();
        scalar->mutable_meta()->set_name("left_grip");
        scalar->mutable_meta()->set_entity("left_gripper");
        scalar->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_LEFT_GRIPPER);
        scalar->mutable_meta()->set_confidence(1.0F);
        scalar->mutable_meta()->set_valid(true);
        scalar->set_value(0.5 + 0.4 * std::sin(tSec));
        scalar->set_min_value(0.0);
        scalar->set_max_value(1.0);

        auto* planar = frame->add_planar_motions();
        planar->mutable_meta()->set_name("base_cmd");
        planar->mutable_meta()->set_entity("mobile_base");
        planar->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_BASE);
        planar->mutable_meta()->set_confidence(1.0F);
        planar->mutable_meta()->set_valid(true);
        planar->set_vx(0.2);
        planar->set_vy(0.0);
        planar->set_wz(0.2 * std::sin(tSec));
        planar->set_reference_frame_id("base_link");

        (*frame->mutable_tags())["demo"] = "embosa_sender";
        return frame;
    }

}  // namespace

int main() {
    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode("puppet_embosa_sender");
    if (node == nullptr) {
        std::cerr << "[sender] failed to create embosa node\n";
        return 1;
    }

    auto writer = node->CreateWriter<PrimitiveFramePb>("puppet_demo/primitive_frame");
    if (writer == nullptr) {
        std::cerr << "[sender] failed to create writer\n";
        return 1;
    }

    uint64_t seq = 0;
    while (galbot::embosa::OK()) {
        const double tSec = static_cast<double>(seq);
        auto frame        = BuildFrame(seq, tSec);
        writer->Publish(frame);
        std::cout << "[sender] publish seq=" << seq << " x=" << frame->poses(0).pose().position().x() << '\n';
        ++seq;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
