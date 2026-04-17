#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "puppet/runtime/teleop_runtime.hpp"

namespace {

    puppet::model::PrimitiveFrame buildMockFrame(uint64_t sequenceId, const std::vector<std::string>& humanBodies) {
        puppet::model::PrimitiveFrame frame;
        frame.sequenceId       = sequenceId;
        frame.context.sourceId = "mock_source";
        frame.context.mode     = "direct";

        for (size_t i = 0; i < humanBodies.size(); ++i) {
            puppet::model::PosePrimitive pose;
            pose.meta.entity           = humanBodies[i];
            pose.meta.referenceFrameId = "world";
            pose.meta.confidence       = 1.0F;
            pose.pose.position.x       = 0.0;
            pose.pose.position.y       = (i % 2 == 0) ? 0.1 : -0.1;
            pose.pose.position.z       = 1.0 - static_cast<double>(i) * 0.02;
            pose.pose.orientation.w    = 1.0;
            frame.poses.push_back(std::move(pose));
        }

        puppet::model::JointCommandPrimitive joint;
        joint.meta.entity = "right_arm";
        joint.jointNames  = {"r_shoulder_pitch", "r_shoulder_roll", "r_elbow"};
        joint.position    = {0.1, -0.2, 0.3};
        frame.jointCommands.push_back(joint);

        return frame;
    }

}  // namespace

int main(int argc, char** argv) {
    std::string configPath = "config/runtime/teleop_runtime.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    puppet::runtime::TeleopRuntime runtime;
    std::string error;
    if (!runtime.init(configPath, error)) {
        std::cerr << "init failed: " << error << std::endl;
        return 1;
    }

    const auto& humanBodies = runtime.config().mockHumanBodies;
    if (!runtime.config().enableMockSource) {
        std::cout << "[teleop_runtime_main] enable_mock_source=false, exit without mock publish" << std::endl;
        return 0;
    }

    for (uint64_t i = 1; i <= 50; ++i) {
        runtime.sourceManager()->ingestFrame(buildMockFrame(i, humanBodies));
        if (!runtime.runOnce(error)) {
            std::cerr << "runOnce failed: " << error << std::endl;
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
