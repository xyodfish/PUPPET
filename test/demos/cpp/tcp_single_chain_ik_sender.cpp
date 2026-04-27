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

namespace puppet::test::tcp_single_chain_ik_sender_detail {

    std::atomic<bool> gStopRequested{false};

    void signalHandler(int) {
        gStopRequested.store(true);
    }

    bool sendAll(int fd, const uint8_t* data, size_t size) {
        size_t sent = 0;
        while (sent < size) {
            const ssize_t rc = send(fd, data + sent, size - sent, 0);
            if (rc <= 0) {
                return false;
            }
            sent += static_cast<size_t>(rc);
        }
        return true;
    }

}  // namespace puppet::test::tcp_single_chain_ik_sender_detail

int main(int argc, char** argv) {
    using namespace puppet::test::tcp_single_chain_ik_sender_detail;
    using puppet::test::single_chain_ik_demo_frame_builder::buildFrame;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string bindIp = "0.0.0.0";
    uint16_t bindPort  = 16661;
    if (argc > 1) {
        bindIp = argv[1];
    }
    if (argc > 2) {
        bindPort = static_cast<uint16_t>(std::stoi(argv[2]));
    }

    int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        std::cerr << "[tcp_single_chain_ik_sender] create socket failed: " << std::strerror(errno) << "\n";
        return 1;
    }

    int enableReuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enableReuse, sizeof(enableReuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(bindPort);
    if (inet_pton(AF_INET, bindIp.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "[tcp_single_chain_ik_sender] invalid bind ip: " << bindIp << "\n";
        close(serverFd);
        return 1;
    }

    if (bind(serverFd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "[tcp_single_chain_ik_sender] bind failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }
    if (listen(serverFd, 1) != 0) {
        std::cerr << "[tcp_single_chain_ik_sender] listen failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }

    std::cout << "[tcp_single_chain_ik_sender] listening on " << bindIp << ":" << bindPort << "\n";
    const int connFd = accept(serverFd, nullptr, nullptr);
    if (connFd < 0) {
        std::cerr << "[tcp_single_chain_ik_sender] accept failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }
    std::cout << "[tcp_single_chain_ik_sender] runtime connected\n";

    uint64_t seq         = 0;
    const double dt      = 0.02;
    const auto sleepTime = std::chrono::milliseconds(20);
    while (!gStopRequested.load()) {
        const double tSec = static_cast<double>(seq) * dt;
        auto frame        = buildFrame(seq, tSec);

        std::string payload(frame.ByteSizeLong(), '\0');
        if (!frame.SerializeToArray(payload.data(), static_cast<int>(payload.size()))) {
            std::cerr << "[tcp_single_chain_ik_sender] serialize frame failed\n";
            break;
        }

        const uint32_t netLen = htonl(static_cast<uint32_t>(payload.size()));
        if (!sendAll(connFd, reinterpret_cast<const uint8_t*>(&netLen), sizeof(netLen)) ||
            !sendAll(connFd, reinterpret_cast<const uint8_t*>(payload.data()), payload.size())) {
            std::cerr << "[tcp_single_chain_ik_sender] send failed\n";
            break;
        }

        if ((seq % 25U) == 0U) {
            std::cout << "[tcp_single_chain_ik_sender] seq=" << seq << "\n";
        }
        ++seq;
        std::this_thread::sleep_for(sleepTime);
    }

    close(connFd);
    close(serverFd);
    return 0;
}
