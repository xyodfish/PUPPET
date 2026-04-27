#include <iostream>
#include <string>

#include "puppet/device/device_service.hpp"
#include "puppet/device/device_service_config.hpp"

int main(int argc, char** argv) {
    std::string configPath = "config/device/device_service_static_file_embosa.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    puppet::device::DeviceServiceConfig config;
    std::string error;
    if (!puppet::device::loadDeviceServiceConfig(configPath, &config, &error)) {
        std::cerr << "[device_service] load config failed: " << error << std::endl;
        return 1;
    }

    puppet::device::DeviceService service;
    if (!service.initialize(config, &error)) {
        std::cerr << "[device_service] initialize failed: " << error << std::endl;
        return 2;
    }

    return service.run();
}
