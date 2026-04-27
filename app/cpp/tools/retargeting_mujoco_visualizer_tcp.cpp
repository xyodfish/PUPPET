#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "puppet/control_intent.pb.h"

namespace puppet::tools::retargeting_mujoco_visualizer_tcp_internal {

    struct VisualizerConfig {
        std::string mujocoModelXml;
        std::string robotName      = "unitree_g1";
        std::string cameraBodyName = "pelvis";
        double cameraDistance      = 2.0;
        double cameraAzimuth       = 0.0;
        bool hasCameraAzimuth      = false;
        int transparentRobot       = 0;
        int width                  = 1280;
        int height                 = 720;
        int renderHz               = 60;
        bool printStat             = true;

        std::string controlIntentBindIp = "0.0.0.0";
        uint16_t controlIntentBindPort  = 16663;
    };

    bool loadConfig(const std::string& path, VisualizerConfig* cfg, std::string* error) {
        try {
            YAML::Node root = YAML::LoadFile(path);
            if (root["mujoco_model_xml"]) {
                cfg->mujocoModelXml = root["mujoco_model_xml"].as<std::string>();
            }
            if (root["robot_name"]) {
                cfg->robotName = root["robot_name"].as<std::string>();
            }
            if (root["camera_body_name"]) {
                cfg->cameraBodyName = root["camera_body_name"].as<std::string>();
            }
            if (root["camera_distance"]) {
                cfg->cameraDistance = root["camera_distance"].as<double>();
            }
            if (root["camera_azimuth"]) {
                cfg->cameraAzimuth    = root["camera_azimuth"].as<double>();
                cfg->hasCameraAzimuth = true;
            }
            if (root["transparent_robot"]) {
                cfg->transparentRobot = root["transparent_robot"].as<int>();
            }
            if (root["window_width"]) {
                cfg->width = root["window_width"].as<int>();
            }
            if (root["window_height"]) {
                cfg->height = root["window_height"].as<int>();
            }
            if (root["render_hz"]) {
                cfg->renderHz = root["render_hz"].as<int>();
            }
            if (root["print_stat"]) {
                cfg->printStat = root["print_stat"].as<bool>();
            }
            if (root["control_intent_bind_ip"]) {
                cfg->controlIntentBindIp = root["control_intent_bind_ip"].as<std::string>();
            }
            if (root["control_intent_bind_port"]) {
                cfg->controlIntentBindPort = root["control_intent_bind_port"].as<uint16_t>();
            }

            if (cfg->mujocoModelXml.empty()) {
                *error = "mujoco_model_xml is empty";
                return false;
            }
            return true;
        } catch (const std::exception& ex) {
            *error = ex.what();
            return false;
        }
    }

    class ControlIntentCache {
       public:
        void update(const ::puppet::puppet_proto::ControlIntent& msg) {
            std::lock_guard<std::mutex> lk(mutex_);
            ++messageCount_;
            lastSeq_ = msg.sequence_id();

            for (const auto& groupIntent : msg.group_intents()) {
                for (const auto& jointIntent : groupIntent.joint_command_intents()) {
                    const int n = std::min(jointIntent.joint_names_size(), jointIntent.position_size());
                    for (int i = 0; i < n; ++i) {
                        jointPosition_[jointIntent.joint_names(i)] = jointIntent.position(i);
                    }
                }
            }
        }

        std::unordered_map<std::string, double> snapshot(int64_t* messageCount, uint64_t* lastSeq) const {
            std::lock_guard<std::mutex> lk(mutex_);
            if (messageCount != nullptr) {
                *messageCount = messageCount_;
            }
            if (lastSeq != nullptr) {
                *lastSeq = lastSeq_;
            }
            return jointPosition_;
        }

       private:
        mutable std::mutex mutex_;
        std::unordered_map<std::string, double> jointPosition_;
        int64_t messageCount_ = 0;
        uint64_t lastSeq_     = 0;
    };

    bool recvAll(int fd, uint8_t* data, size_t size) {
        size_t received = 0;
        while (received < size) {
            const ssize_t rc = recv(fd, data + received, size - received, 0);
            if (rc <= 0) {
                return false;
            }
            received += static_cast<size_t>(rc);
        }
        return true;
    }

}  // namespace puppet::tools::retargeting_mujoco_visualizer_tcp_internal

int main(int argc, char** argv) {
    using namespace puppet::tools::retargeting_mujoco_visualizer_tcp_internal;

    std::string configPath = "config/tools/demo_single_chain_ik_visualizer_tcp.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    VisualizerConfig cfg;
    std::string error;
    if (!loadConfig(configPath, &cfg, &error)) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] load config failed: " << error << std::endl;
        return 1;
    }

    char modelError[1024] = {0};
    mjModel* modelRaw     = mj_loadXML(cfg.mujocoModelXml.c_str(), nullptr, modelError, sizeof(modelError));
    if (modelRaw == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] load model failed: " << modelError << std::endl;
        return 2;
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(modelRaw, &mj_deleteModel);

    mjData* dataRaw = mj_makeData(model.get());
    if (dataRaw == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] make data failed" << std::endl;
        return 3;
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(dataRaw, &mj_deleteData);

    if (!glfwInit()) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] glfw init failed" << std::endl;
        return 4;
    }

    GLFWwindow* window = glfwCreateWindow(cfg.width, cfg.height, "PUPPET Viewer TCP", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] create window failed" << std::endl;
        glfwTerminate();
        return 5;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjv_defaultCamera(&camera);
    mjv_defaultOption(&option);
    mjv_defaultScene(&scene);
    mjr_defaultContext(&context);
    mjv_makeScene(model.get(), &scene, 5000);
    mjr_makeContext(model.get(), &context, mjFONTSCALE_150);
    option.flags[mjVIS_TRANSPARENT] = cfg.transparentRobot;

    camera.type      = mjCAMERA_TRACKING;
    camera.distance  = cfg.cameraDistance;
    camera.elevation = -10.0;
    if (cfg.hasCameraAzimuth) {
        camera.azimuth = cfg.cameraAzimuth;
    }

    int cameraBodyId = mj_name2id(model.get(), mjOBJ_BODY, cfg.cameraBodyName.c_str());
    if (cameraBodyId >= 0) {
        camera.trackbodyid = cameraBodyId;
    }

    std::unordered_map<std::string, int> jointQposAdr;
    for (int j = 0; j < model->njnt; ++j) {
        const int nameAdr = model->name_jntadr[j];
        if (nameAdr < 0) {
            continue;
        }
        const std::string name(model->names + nameAdr);
        const int type = model->jnt_type[j];
        if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
            jointQposAdr[name] = model->jnt_qposadr[j];
        }
    }

    int serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if (serverFd < 0) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] create socket failed: " << std::strerror(errno) << std::endl;
        return 6;
    }
    int enableReuse = 1;
    setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &enableReuse, sizeof(enableReuse));

    sockaddr_in bindAddr{};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_port   = htons(cfg.controlIntentBindPort);
    if (inet_pton(AF_INET, cfg.controlIntentBindIp.c_str(), &bindAddr.sin_addr) != 1) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] invalid bind ip: " << cfg.controlIntentBindIp << std::endl;
        close(serverFd);
        return 7;
    }
    if (bind(serverFd, reinterpret_cast<const sockaddr*>(&bindAddr), sizeof(bindAddr)) != 0) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] bind failed: " << std::strerror(errno) << std::endl;
        close(serverFd);
        return 8;
    }
    if (listen(serverFd, 1) != 0) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] listen failed: " << std::strerror(errno) << std::endl;
        close(serverFd);
        return 9;
    }
    std::cout << "[retargeting_mujoco_visualizer_tcp] waiting runtime on " << cfg.controlIntentBindIp << ":" << cfg.controlIntentBindPort
              << std::endl;
    int connFd = accept(serverFd, nullptr, nullptr);
    if (connFd < 0) {
        std::cerr << "[retargeting_mujoco_visualizer_tcp] accept failed: " << std::strerror(errno) << std::endl;
        close(serverFd);
        return 10;
    }
    std::cout << "[retargeting_mujoco_visualizer_tcp] runtime connected" << std::endl;

    ControlIntentCache cache;
    const int renderHz = cfg.renderHz > 0 ? cfg.renderHz : 60;
    const auto sleepDt = std::chrono::milliseconds(1000 / renderHz);

    while (!glfwWindowShouldClose(window)) {
        uint32_t netLen = 0;
        if (!recvAll(connFd, reinterpret_cast<uint8_t*>(&netLen), sizeof(netLen))) {
            break;
        }
        const uint32_t payloadLen = ntohl(netLen);
        if (payloadLen == 0 || payloadLen > 32U * 1024U * 1024U) {
            std::cerr << "[retargeting_mujoco_visualizer_tcp] invalid payload length: " << payloadLen << std::endl;
            break;
        }
        std::string payload(payloadLen, '\0');
        if (!recvAll(connFd, reinterpret_cast<uint8_t*>(payload.data()), payload.size())) {
            break;
        }

        ::puppet::puppet_proto::ControlIntent msg;
        if (msg.ParseFromArray(payload.data(), static_cast<int>(payload.size()))) {
            cache.update(msg);
        }

        int64_t messageCount = 0;
        uint64_t lastSeq     = 0;
        const auto jointMap  = cache.snapshot(&messageCount, &lastSeq);

        mju_copy(data->qpos, model->qpos0, model->nq);
        int mappedCount = 0;
        for (const auto& kv : jointMap) {
            const auto it = jointQposAdr.find(kv.first);
            if (it == jointQposAdr.end()) {
                continue;
            }
            data->qpos[it->second] = kv.second;
            ++mappedCount;
        }

        for (int j = 0; j < model->njnt; ++j) {
            const int jointType = model->jnt_type[j];
            if ((jointType == mjJNT_HINGE || jointType == mjJNT_SLIDE) && model->jnt_limited[j] > 0) {
                const int qadr    = model->jnt_qposadr[j];
                const double qmin = model->jnt_range[2 * j + 0];
                const double qmax = model->jnt_range[2 * j + 1];
                data->qpos[qadr]  = std::min(std::max(data->qpos[qadr], qmin), qmax);
            }
        }

        mj_forward(model.get(), data.get());
        if (cameraBodyId >= 0) {
            camera.lookat[0] = data->xpos[3 * cameraBodyId + 0];
            camera.lookat[1] = data->xpos[3 * cameraBodyId + 1];
            camera.lookat[2] = data->xpos[3 * cameraBodyId + 2];
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(model.get(), data.get(), &option, nullptr, &camera, mjCAT_ALL, &scene);
        mjr_render(viewport, &scene, &context);

        if (cfg.printStat) {
            static int64_t lastPrintCount = -1;
            if (messageCount != lastPrintCount) {
                lastPrintCount = messageCount;
                std::cout << "[retargeting_mujoco_visualizer_tcp] msg_count=" << messageCount << " last_seq=" << lastSeq
                          << " mapped_joints=" << mappedCount << std::endl;
            }
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(sleepDt);
    }

    close(connFd);
    close(serverFd);
    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
