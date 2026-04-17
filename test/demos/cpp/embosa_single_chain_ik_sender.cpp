#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "embosa.hpp"
#include "puppet/primitive_frame.pb.h"
#include "puppet/uniform.pb.h"

namespace {

    using PrimitiveFramePb = ::puppet::puppet_proto::PrimitiveFrame;
    using BodyGroupPb      = ::puppet::puppet_proto::BodyGroup;
    using SourceTypePb     = ::puppet::puppet_proto::SourceType;

    double squareWaveUnit(double phase) {
        const double p = std::fmod(phase, 1.0);
        if (p < 0.25) {
            return p / 0.25;
        }
        if (p < 0.5) {
            return 1.0;
        }
        if (p < 0.75) {
            return 1.0 - (p - 0.5) / 0.25;
        }
        return 0.0;
    }

    std::shared_ptr<PrimitiveFramePb> buildFrame(uint64_t sequence_id, double t_sec) {
        auto frame = std::make_shared<PrimitiveFramePb>();

        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->mutable_header()->mutable_timestamp()->set_sec(sec);
        frame->mutable_header()->mutable_timestamp()->set_nanosec(static_cast<uint32_t>(nsec));
        frame->mutable_header()->set_frame_id("torso_link");

        frame->set_sequence_id(sequence_id);
        frame->mutable_context()->set_source_id("demo_sender_single_chain_ik");
        frame->mutable_context()->set_source_type(SourceTypePb::SOURCE_TYPE_EXTERNAL);
        frame->mutable_context()->set_semantic_context("single_chain_ik_demo");
        frame->mutable_context()->set_mode("cart_pose_to_joint");
        frame->mutable_context()->set_robot_id("unitree_g1");
        frame->mutable_context()->set_pipeline_id("single_chain_ik_pipeline");

        const double omega = 0.8;
        // Target in torso_link frame (same base frame as TRAC-IK chain base_link).
        const double cx  = 0.26;
        const double cy  = -0.18;
        const double cz  = 0.08;
        const double rx  = 0.03;
        const double rz  = 0.02;
        const double yaw = 0.15 * std::sin(omega * t_sec);

        auto* pose = frame->add_poses();
        pose->mutable_meta()->set_name("right_wrist_pose_demo");
        pose->mutable_meta()->set_entity("right_wrist");
        pose->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_RIGHT_ARM);
        pose->mutable_meta()->set_frame_id("torso_link");
        pose->mutable_meta()->set_reference_frame_id("torso_link");
        pose->mutable_meta()->set_confidence(1.0F);
        pose->mutable_meta()->set_valid(true);
        pose->mutable_pose()->mutable_position()->set_x(cx + rx * std::cos(omega * t_sec));
        pose->mutable_pose()->mutable_position()->set_y(cy + rx * std::sin(omega * t_sec));
        pose->mutable_pose()->mutable_position()->set_z(cz);
        pose->mutable_pose()->mutable_orientation()->set_x(0.0);
        pose->mutable_pose()->mutable_orientation()->set_y(0.0);
        pose->mutable_pose()->mutable_orientation()->set_z(std::sin(0.5 * yaw));
        pose->mutable_pose()->mutable_orientation()->set_w(std::cos(0.5 * yaw));
        pose->set_is_relative(false);
        pose->set_target_frame_id("torso_link");

        // Left hand: square trajectory in torso_link frame.
        const double square_period = 4.0;
        const double phase         = std::fmod(t_sec / square_period, 1.0);
        const double edge          = 0.05;
        double left_x              = 0.24;
        double left_y              = 0.18;
        double left_z              = 0.10;

        if (phase < 0.25) {
            left_x += edge * (phase / 0.25);
            left_z += edge;
        } else if (phase < 0.5) {
            left_x += edge;
            left_z += edge * (1.0 - (phase - 0.25) / 0.25);
        } else if (phase < 0.75) {
            left_x += edge * (1.0 - (phase - 0.5) / 0.25);
        } else {
            left_z += edge * ((phase - 0.75) / 0.25);
        }
        const double left_yaw = 0.12 * (2.0 * squareWaveUnit(phase) - 1.0);

        auto* left_pose = frame->add_poses();
        left_pose->mutable_meta()->set_name("left_wrist_pose_demo");
        left_pose->mutable_meta()->set_entity("left_wrist");
        left_pose->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_LEFT_ARM);
        left_pose->mutable_meta()->set_frame_id("torso_link");
        left_pose->mutable_meta()->set_reference_frame_id("torso_link");
        left_pose->mutable_meta()->set_confidence(1.0F);
        left_pose->mutable_meta()->set_valid(true);
        left_pose->mutable_pose()->mutable_position()->set_x(left_x);
        left_pose->mutable_pose()->mutable_position()->set_y(left_y);
        left_pose->mutable_pose()->mutable_position()->set_z(left_z);
        left_pose->mutable_pose()->mutable_orientation()->set_x(0.0);
        left_pose->mutable_pose()->mutable_orientation()->set_y(0.0);
        left_pose->mutable_pose()->mutable_orientation()->set_z(std::sin(0.5 * left_yaw));
        left_pose->mutable_pose()->mutable_orientation()->set_w(std::cos(0.5 * left_yaw));
        left_pose->set_is_relative(false);
        left_pose->set_target_frame_id("torso_link");

        auto* joint_state = frame->add_joint_states();
        joint_state->mutable_meta()->set_name("right_arm_seed_state");
        joint_state->mutable_meta()->set_entity("right_arm");
        joint_state->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_RIGHT_ARM);
        joint_state->mutable_meta()->set_frame_id("torso_link");
        joint_state->mutable_meta()->set_reference_frame_id("torso_link");
        joint_state->mutable_meta()->set_confidence(1.0F);
        joint_state->mutable_meta()->set_valid(true);

        const std::vector<std::string> joint_names = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint",     "right_wrist_pitch_joint",   "right_wrist_yaw_joint",
        };
        const std::vector<double> seed_positions = {0.2, -0.15, 0.0, 0.45, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < joint_names.size(); ++i) {
            joint_state->add_joint_names(joint_names[i]);
            joint_state->add_position(seed_positions[i]);
        }

        auto* left_joint_state = frame->add_joint_states();
        left_joint_state->mutable_meta()->set_name("left_arm_seed_state");
        left_joint_state->mutable_meta()->set_entity("left_arm");
        left_joint_state->mutable_meta()->set_body_group(BodyGroupPb::BODY_GROUP_LEFT_ARM);
        left_joint_state->mutable_meta()->set_frame_id("torso_link");
        left_joint_state->mutable_meta()->set_reference_frame_id("torso_link");
        left_joint_state->mutable_meta()->set_confidence(1.0F);
        left_joint_state->mutable_meta()->set_valid(true);

        const std::vector<std::string> left_joint_names = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint",     "left_wrist_pitch_joint",   "left_wrist_yaw_joint",
        };
        const std::vector<double> left_seed_positions = {0.2, 0.15, 0.0, 0.45, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < left_joint_names.size(); ++i) {
            left_joint_state->add_joint_names(left_joint_names[i]);
            left_joint_state->add_position(left_seed_positions[i]);
        }

        (*frame->mutable_tags())["demo"] = "single_chain_ik";
        return frame;
    }

}  // namespace

int main() {
    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode("puppet_single_chain_ik_sender");
    if (node == nullptr) {
        std::cerr << "[single_chain_ik_sender] failed to create embosa node\n";
        return 1;
    }

    auto writer = node->CreateWriter<PrimitiveFramePb>("puppet_demo/primitive_frame");
    if (writer == nullptr) {
        std::cerr << "[single_chain_ik_sender] failed to create writer\n";
        return 1;
    }

    uint64_t seq       = 0;
    const double dt    = 0.02;
    const auto sleep_t = std::chrono::milliseconds(20);
    while (galbot::embosa::OK()) {
        const double t_sec = static_cast<double>(seq) * dt;
        auto frame         = buildFrame(seq, t_sec);
        writer->Publish(frame);

        if ((seq % 25U) == 0U) {
            std::cout << "[single_chain_ik_sender] seq=" << seq << " right=(" << frame->poses(0).pose().position().x() << ", "
                      << frame->poses(0).pose().position().y() << ", " << frame->poses(0).pose().position().z() << ")"
                      << " left=(" << frame->poses(1).pose().position().x() << ", " << frame->poses(1).pose().position().y() << ", "
                      << frame->poses(1).pose().position().z() << ")\n";
        }

        ++seq;
        std::this_thread::sleep_for(sleep_t);
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
