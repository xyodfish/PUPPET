#include <iostream>
#include <string>

#include <glog/logging.h>
#include <spdlog/spdlog.h>

#include "puppet/runtime/puppet_manager.hpp"

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    std::string runtimeConfig = "config/runtime/teleop_runtime_gmr.yaml";
    if (argc > 1) {
        runtimeConfig = argv[1];
    }
    LOG(INFO) << "teleop_runtime_embosa_main start config=" << runtimeConfig;

    puppet::runtime::PuppetManager manager;
    std::string error;
    if (!manager.init(runtimeConfig, error)) {
        std::cerr << "[teleop_runtime_embosa] init failed: " << error << std::endl;
        LOG(ERROR) << "teleop runtime embosa init failed: " << error;
        google::ShutdownGoogleLogging();
        return 1;
    }

    if (!manager.run(error)) {
        std::cerr << "[teleop_runtime_embosa] run failed: " << error << std::endl;
        LOG(ERROR) << "teleop runtime embosa run failed: " << error;
        google::ShutdownGoogleLogging();
        return 2;
    }

    google::ShutdownGoogleLogging();
    return 0;
}
