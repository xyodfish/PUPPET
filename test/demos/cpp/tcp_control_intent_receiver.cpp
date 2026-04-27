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

namespace puppet::test::tcp_control_intent_receiver_detail {

    std::atomic<bool> gStopRequested{false};

    void signalHandler(int) {
        gStopRequested.store(true);
    }

    bool recvAll(int fd, uint8_t* out, size_t size) {
        size_t received = 0;
        while (received < size && !gStopRequested.load()) {
            const ssize_t rc = recv(fd, out + received, size - received, 0);
            if (rc <= 0) {
                return false;
            }
            received += static_cast<size_t>(rc);
        }
        return received == size;
    }

}  // namespace puppet::test::tcp_control_intent_receiver_detail

int main(int argc, char** argv) {
    using namespace puppet::test::tcp_control_intent_receiver_detail;

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

    int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        std::cerr << "[tcp_control_intent_receiver] create socket failed: " << std::strerror(errno) << "\n";
        return 1;
    }

    int enableReuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enableReuse, sizeof(enableReuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(bindPort);
    if (inet_pton(AF_INET, bindIp.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "[tcp_control_intent_receiver] invalid bind ip: " << bindIp << "\n";
        close(serverFd);
        return 1;
    }

    if (bind(serverFd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        std::cerr << "[tcp_control_intent_receiver] bind failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }
    if (listen(serverFd, 1) != 0) {
        std::cerr << "[tcp_control_intent_receiver] listen failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }

    std::cout << "[tcp_control_intent_receiver] listening on " << bindIp << ":" << bindPort << "\n";
    const int connFd = accept(serverFd, nullptr, nullptr);
    if (connFd < 0) {
        std::cerr << "[tcp_control_intent_receiver] accept failed: " << std::strerror(errno) << "\n";
        close(serverFd);
        return 1;
    }
    std::cout << "[tcp_control_intent_receiver] runtime connected\n";

    uint64_t recvCount = 0;
    while (!gStopRequested.load()) {
        uint32_t netLen = 0;
        if (!recvAll(connFd, reinterpret_cast<uint8_t*>(&netLen), sizeof(netLen))) {
            break;
        }
        const uint32_t payloadLen = ntohl(netLen);
        if (payloadLen == 0 || payloadLen > 32U * 1024U * 1024U) {
            std::cerr << "[tcp_control_intent_receiver] invalid payload length: " << payloadLen << "\n";
            break;
        }

        std::vector<uint8_t> payload(payloadLen);
        if (!recvAll(connFd, payload.data(), payload.size())) {
            break;
        }

        ::puppet::puppet_proto::ControlIntent intentPb;
        if (!intentPb.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
            std::cerr << "[tcp_control_intent_receiver] parse ControlIntent failed\n";
            continue;
        }

        ++recvCount;
        if ((recvCount % 25U) == 0U) {
            std::cout << "[tcp_control_intent_receiver] count=" << recvCount << " sequence_id=" << intentPb.sequence_id()
                      << " groups=" << intentPb.group_intents_size() << "\n";
        }
    }

    close(connFd);
    close(serverFd);
    return 0;
}
