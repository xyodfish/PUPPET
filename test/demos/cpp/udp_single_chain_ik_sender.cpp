#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "single_chain_ik_demo_frame_builder.hpp"

namespace puppet::test::udp_single_chain_ik_sender_detail {

    std::atomic<bool> gStopRequested{false};

    void signalHandler(int) {
        gStopRequested.store(true);
    }

}  // namespace puppet::test::udp_single_chain_ik_sender_detail

int main(int argc, char** argv) {
    using namespace puppet::test::udp_single_chain_ik_sender_detail;
    using puppet::test::single_chain_ik_demo_frame_builder::buildFrame;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string remoteIp = "127.0.0.1";
    uint16_t remotePort  = 16661;
    if (argc > 1) {
        remoteIp = argv[1];
    }
    if (argc > 2) {
        remotePort = static_cast<uint16_t>(std::stoi(argv[2]));
    }

    int sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockFd < 0) {
        std::cerr << "[udp_single_chain_ik_sender] create socket failed: " << std::strerror(errno) << "\n";
        return 1;
    }

    sockaddr_in remoteAddr{};
    remoteAddr.sin_family = AF_INET;
    remoteAddr.sin_port   = htons(remotePort);
    if (inet_pton(AF_INET, remoteIp.c_str(), &remoteAddr.sin_addr) != 1) {
        std::cerr << "[udp_single_chain_ik_sender] invalid remote ip: " << remoteIp << "\n";
        close(sockFd);
        return 1;
    }

    uint64_t seq         = 0;
    const double dt      = 0.02;
    const auto sleepTime = std::chrono::milliseconds(20);
    while (!gStopRequested.load()) {
        const double tSec = static_cast<double>(seq) * dt;
        auto frame        = buildFrame(seq, tSec);

        std::string payload(frame.ByteSizeLong(), '\0');
        if (!frame.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
            std::cerr << "[udp_single_chain_ik_sender] serialize frame failed\n";
            break;
        }

        const ssize_t rc =
            sendto(sockFd, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr*>(&remoteAddr), sizeof(remoteAddr));
        if (rc < 0) {
            std::cerr << "[udp_single_chain_ik_sender] sendto failed: " << std::strerror(errno) << "\n";
            break;
        }

        if ((seq % 25U) == 0U) {
            std::cout << "[udp_single_chain_ik_sender] seq=" << seq << "\n";
        }
        ++seq;
        std::this_thread::sleep_for(sleepTime);
    }

    close(sockFd);
    return 0;
}
