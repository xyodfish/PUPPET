#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "embosa.hpp"

#include "puppet/primitive_frame.pb.h"
#include "puppet/transport/proto_copy.hpp"

namespace {

    void OnPrimitiveFrame(const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg) {
        if (msg == nullptr) {
            std::cerr << "[receiver] null message\n";
            return;
        }

        puppet::model::PrimitiveFrame modelFrame;
        if (!puppet::transport::copyFromProto(*msg, &modelFrame)) {
            std::cerr << "[receiver] proto copy failed\n";
            return;
        }

        std::cout << "[receiver] seq=" << modelFrame.sequenceId << " source=" << modelFrame.context.sourceId
                  << " mode=" << modelFrame.context.mode << " poses=" << modelFrame.poses.size() << " scalars=" << modelFrame.scalars.size()
                  << " baseCmds=" << modelFrame.planarMotions.size() << '\n';

        if (!modelFrame.poses.empty()) {
            const auto& pose = modelFrame.poses.front();
            std::cout << "  pose[" << pose.meta.name << "] "
                      << "x=" << pose.pose.position.x << ", y=" << pose.pose.position.y << ", z=" << pose.pose.position.z << '\n';
        }
        if (!modelFrame.scalars.empty()) {
            const auto& scalar = modelFrame.scalars.front();
            std::cout << "  scalar[" << scalar.meta.name << "] "
                      << "value=" << scalar.value << '\n';
        }
    }

}  // namespace

int main() {
    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode("puppet_embosa_receiver");
    if (node == nullptr) {
        std::cerr << "[receiver] failed to create embosa node\n";
        return 1;
    }

    auto reader = node->CreateReader<::puppet::puppet_proto::PrimitiveFrame>("puppet_demo/primitive_frame",
                                                                             std::bind(&OnPrimitiveFrame, std::placeholders::_1));
    if (reader == nullptr) {
        std::cerr << "[receiver] failed to create reader\n";
        return 1;
    }

    while (galbot::embosa::OK()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();
    return 0;
}
