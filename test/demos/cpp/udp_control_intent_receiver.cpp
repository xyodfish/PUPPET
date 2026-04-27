#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "puppet/control_intent.pb.h"

namespace puppet::test::udp_control_intent_receiver_detail {

    std::atomic<bool> gStopRequested{false};

    void signalHandler(int) {
        gStopRequested.store(true);
    }

}  // namespace puppet::test::udp_control_intent_receiver_detail

int main(int argc, char** argv) {
    using namespace puppet::test::udp_control_intent_receiver_detail;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string bindIp = "0.0.0.0";
    uint16_t bindPort  = 16663;
    if (argc > 1) {
        bindIp = argv[1];
    }
    if (argc > 2) {
        bindPort = static_cast<uint16_t>(std::stoi(argv[2]));
    }

    int sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockFd < 0) {
        std::cerr << "[udp_control_intent_receiver] create socket failed: " << std::strerror(errno) << "\n";
        return 1;
    }

    int enableReuse = 1;
    setsockopt(sockFd, SOL_SOCKET, SO_REUSEADDR, &enableReuse, sizeof(enableReuse));

    sockaddr_in bindAddr{};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_port   = htons(bindPort);
    if (inet_pton(AF_INET, bindIp.c_str(), &bindAddr.sin_addr) != 1) {
        std::cerr << "[udp_control_intent_receiver] invalid bind ip: " << bindIp << "\n";
        close(sockFd);
        return 1;
    }

    if (bind(sockFd, reinterpret_cast<const sockaddr*>(&bindAddr), sizeof(bindAddr)) != 0) {
        std::cerr << "[udp_control_intent_receiver] bind failed: " << std::strerror(errno) << "\n";
        close(sockFd);
        return 1;
    }
    std::cout << "[udp_control_intent_receiver] listening on " << bindIp << ":" << bindPort << "\n";

    std::vector<uint8_t> payload(65536);
    uint64_t recvCount = 0;
    while (!gStopRequested.load()) {
        sockaddr_in peer{};
        socklen_t peerLen = sizeof(peer);
        const ssize_t rc  = recvfrom(sockFd, payload.data(), payload.size(), 0, reinterpret_cast<sockaddr*>(&peer), &peerLen);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[udp_control_intent_receiver] recvfrom failed: " << std::strerror(errno) << "\n";
            break;
        }
        if (rc == 0) {
            continue;
        }

        ::puppet::puppet_proto::ControlIntent intentPb;
        if (!intentPb.ParseFromArray(payload.data(), static_cast<int>(rc))) {
            std::cerr << "[udp_control_intent_receiver] parse ControlIntent failed\n";
            continue;
        }

        ++recvCount;
        if ((recvCount % 25U) == 0U) {
            std::cout << "[udp_control_intent_receiver] count=" << recvCount << " sequence_id=" << intentPb.sequence_id()
                      << " groups=" << intentPb.group_intents_size() << "\n";
        }
    }

    close(sockFd);
    return 0;
}
