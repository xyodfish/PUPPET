#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <random>
#include <thread>
#include <trac_ik/trac_ik.hpp>
#include <vector>

#include "embosa.hpp"
#include "puppet/primitive_frame.pb.h"
#include "puppet/uniform.pb.h"

namespace {

    using PrimitiveFramePb   = ::puppet::puppet_proto::PrimitiveFrame;
    using JointCommandModePb = ::puppet::puppet_proto::JointCommandPrimitive_JointCommandMode;
    using SourceTypePb       = ::puppet::puppet_proto::SourceType;

    struct ArmFkContext {
        std::unique_ptr<TRAC_IK::TRAC_IK> solver;
        KDL::Chain chain;
        std::vector<std::string> jointNames;
    };

    bool initRightArmFkContext(ArmFkContext* ctx) {
        if (ctx == nullptr) {
            return false;
        }
        ctx->jointNames = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint",     "right_wrist_pitch_joint",   "right_wrist_yaw_joint",
        };
        ctx->solver =
            std::make_unique<TRAC_IK::TRAC_IK>("torso_link", "right_wrist_yaw_link", "assets/unitree_g1/g1_custom_collision_29dof.urdf",
                                               10000, 0.003, 1e-5, false, false, false, TRAC_IK::Speed);
        return ctx->solver->getKDLChain(ctx->chain);
    }

    std::vector<double> makeRightArmJointPositions(double t_sec) {
        return {
            0.35 + 0.15 * std::sin(0.7 * t_sec), -0.20 + 0.10 * std::sin(0.5 * t_sec + 0.6),
            0.10 * std::sin(0.4 * t_sec),        0.70 + 0.20 * std::sin(0.9 * t_sec + 0.3),
            0.10 * std::sin(1.1 * t_sec),        0.10 * std::sin(0.8 * t_sec + 0.8),
            0.10 * std::sin(0.6 * t_sec + 1.2),
        };
    }

    KDL::Frame computeEndEffectorFrame(const ArmFkContext& ctx, const std::vector<double>& jointPositions) {
        KDL::JntArray joints(jointPositions.size());
        for (size_t i = 0; i < jointPositions.size(); ++i) {
            joints(i) = jointPositions[i];
        }

        KDL::ChainFkSolverPos_recursive fkSolver(ctx.chain);
        KDL::Frame eeFrame;
        fkSolver.JntToCart(joints, eeFrame);
        return eeFrame;
    }

    void fillHeaderAndContext(PrimitiveFramePb* frame, uint64_t sequence_id, const char* mode, const char* pluginId) {
        const auto now  = std::chrono::system_clock::now().time_since_epoch();
        const auto sec  = std::chrono::duration_cast<std::chrono::seconds>(now).count();
        const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() - sec * 1000000000LL;

        frame->mutable_header()->mutable_timestamp()->set_sec(sec);
        frame->mutable_header()->mutable_timestamp()->set_nanosec(static_cast<uint32_t>(nsec));
        frame->mutable_header()->set_frame_id("torso_link");

        frame->set_sequence_id(sequence_id);
        frame->mutable_context()->set_source_id("sourceA");
        frame->mutable_context()->set_source_type(SourceTypePb::SOURCE_TYPE_EXTERNAL);
        frame->mutable_context()->set_semantic_context("dual_source_mixed_demo");
        frame->mutable_context()->set_mode(mode);
        frame->mutable_context()->set_robot_id("unitree_g1");
        frame->mutable_context()->set_plugin_id(pluginId);
    }

    std::shared_ptr<PrimitiveFramePb> buildDirectPassFrame(uint64_t sequence_id, const ArmFkContext& ctx, double t_sec) {
        auto frame = std::make_shared<PrimitiveFramePb>();
        fillHeaderAndContext(frame.get(), sequence_id, "direct_joint_command", "direct_pass_plugin");

        auto* joint_cmd = frame->add_joint_commands();
        joint_cmd->mutable_meta()->set_name("right_arm_joint_command_demo");
        joint_cmd->mutable_meta()->set_entity("right_arm");
        joint_cmd->mutable_meta()->set_body_group("right_arm");
        joint_cmd->mutable_meta()->set_frame_id("torso_link");
        joint_cmd->mutable_meta()->set_reference_frame_id("torso_link");
        joint_cmd->mutable_meta()->set_confidence(1.0F);
        joint_cmd->mutable_meta()->set_valid(true);
        joint_cmd->set_mode(JointCommandModePb::JointCommandPrimitive_JointCommandMode_JOINT_COMMAND_MODE_POSITION);

        const auto jointPositions = makeRightArmJointPositions(t_sec);
        for (size_t i = 0; i < ctx.jointNames.size(); ++i) {
            joint_cmd->add_joint_names(ctx.jointNames[i]);
            joint_cmd->add_position(jointPositions[i]);
        }

        auto* joint_state = frame->add_joint_states();
        joint_state->mutable_meta()->set_name("right_arm_seed_state");
        joint_state->mutable_meta()->set_entity("right_arm");
        joint_state->mutable_meta()->set_body_group("right_arm");
        joint_state->mutable_meta()->set_frame_id("torso_link");
        joint_state->mutable_meta()->set_reference_frame_id("torso_link");
        joint_state->mutable_meta()->set_confidence(1.0F);
        joint_state->mutable_meta()->set_valid(true);
        for (size_t i = 0; i < ctx.jointNames.size(); ++i) {
            joint_state->add_joint_names(ctx.jointNames[i]);
            joint_state->add_position(jointPositions[i]);
        }

        (*frame->mutable_tags())["demo"]     = "dual_source_mixed";
        (*frame->mutable_tags())["source"]   = "sourceA";
        (*frame->mutable_tags())["strategy"] = "direct_pass";
        return frame;
    }

    std::shared_ptr<PrimitiveFramePb> buildSingleChainIkFrame(uint64_t sequence_id, const ArmFkContext& ctx, double t_sec) {
        auto frame = std::make_shared<PrimitiveFramePb>();
        fillHeaderAndContext(frame.get(), sequence_id, "cart_pose_to_joint", "single_chain_ik_plugin");
        const auto jointPositions = makeRightArmJointPositions(t_sec);
        const auto eeFrame        = computeEndEffectorFrame(ctx, jointPositions);

        auto* pose = frame->add_poses();
        pose->mutable_meta()->set_name("right_wrist_pose_demo");
        pose->mutable_meta()->set_entity("right_wrist");
        pose->mutable_meta()->set_body_group("right_arm");
        pose->mutable_meta()->set_frame_id("torso_link");
        pose->mutable_meta()->set_reference_frame_id("torso_link");
        pose->mutable_meta()->set_confidence(1.0F);
        pose->mutable_meta()->set_valid(true);
        pose->mutable_pose()->mutable_position()->set_x(eeFrame.p.x());
        pose->mutable_pose()->mutable_position()->set_y(eeFrame.p.y());
        pose->mutable_pose()->mutable_position()->set_z(eeFrame.p.z());
        double qx = 0.0;
        double qy = 0.0;
        double qz = 0.0;
        double qw = 1.0;
        eeFrame.M.GetQuaternion(qx, qy, qz, qw);
        pose->mutable_pose()->mutable_orientation()->set_x(qx);
        pose->mutable_pose()->mutable_orientation()->set_y(qy);
        pose->mutable_pose()->mutable_orientation()->set_z(qz);
        pose->mutable_pose()->mutable_orientation()->set_w(qw);
        pose->set_is_relative(false);
        pose->set_target_frame_id("torso_link");

        auto* joint_state = frame->add_joint_states();
        joint_state->mutable_meta()->set_name("right_arm_seed_state");
        joint_state->mutable_meta()->set_entity("right_arm");
        joint_state->mutable_meta()->set_body_group("right_arm");
        joint_state->mutable_meta()->set_frame_id("torso_link");
        joint_state->mutable_meta()->set_reference_frame_id("torso_link");
        joint_state->mutable_meta()->set_confidence(1.0F);
        joint_state->mutable_meta()->set_valid(true);
        for (size_t i = 0; i < ctx.jointNames.size(); ++i) {
            joint_state->add_joint_names(ctx.jointNames[i]);
            joint_state->add_position(jointPositions[i]);
        }

        (*frame->mutable_tags())["demo"]     = "dual_source_mixed";
        (*frame->mutable_tags())["source"]   = "sourceA";
        (*frame->mutable_tags())["strategy"] = "single_chain_ik";
        return frame;
    }

}  // namespace

int main() {
    ArmFkContext fkContext;
    if (!initRightArmFkContext(&fkContext)) {
        std::cerr << "[direct_pass_sender] failed to initialize right-arm FK context\n";
        return 1;
    }

    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode("puppet_direct_pass_sender");
    if (node == nullptr) {
        std::cerr << "[direct_pass_sender] failed to create embosa node\n";
        return 1;
    }

    auto writer = node->CreateWriter<PrimitiveFramePb>("puppet_demo/primitive_frame");
    if (writer == nullptr) {
        std::cerr << "[direct_pass_sender] failed to create writer\n";
        return 1;
    }

    uint64_t seq       = 0;
    const double dt    = 0.02;
    const auto sleep_t = std::chrono::milliseconds(20);
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> strategy_dist(0, 1);
    bool useDirectPass = true;
    while (galbot::embosa::OK()) {
        const double t_sec = static_cast<double>(seq) * dt;
        if ((seq % 100U) == 0U) {
            useDirectPass = strategy_dist(rng) == 0;
        }

        auto frame = useDirectPass ? buildDirectPassFrame(seq, fkContext, t_sec) : buildSingleChainIkFrame(seq, fkContext, t_sec);
        writer->Publish(frame);

        if ((seq % 25U) == 0U) {
            std::cout << "[direct_pass_sender] seq=" << seq << " strategy=" << frame->context().plugin_id();
            if (frame->joint_commands_size() > 0) {
                std::cout << " right_elbow=" << frame->joint_commands(0).position(3);
            } else if (frame->poses_size() > 0) {
                std::cout << " right_wrist_x=" << frame->poses(0).pose().position().x();
            }
            std::cout << "\n";
        }

        ++seq;
        std::this_thread::sleep_for(sleep_t);
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
