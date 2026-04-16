#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "embosa.hpp"
#include "puppet/control_intent.pb.h"

namespace puppet::tools::retargeting_mujoco_visualizer_internal {

    struct VisualizerConfig {
        std::string nodeName  = "puppet_retargeting_mujoco_visualizer";
        std::string topicName = "puppet_demo/control_intent";
        std::string mujocoModelXml;
        std::string robotName                     = "unitree_g1";
        std::string cameraBodyName                = "pelvis";
        double cameraDistance                     = 2.5;
        double cameraAzimuth                      = 180.0;
        int transparentRobot                      = 0;
        bool showBodyOverlay                      = true;
        double bodyOverlayScale                   = 0.1;
        std::vector<std::string> overlayBodyNames = {"pelvis"};
        int width                                 = 1280;
        int height                                = 720;
        int renderHz                              = 60;
        bool printStat                            = true;
    };

    const std::unordered_map<std::string, std::string>& robotBaseMap() {
        static const std::unordered_map<std::string, std::string> kMap = {
            {"unitree_g1", "pelvis"},
            {"unitree_g1_with_hands", "pelvis"},
            {"unitree_h1", "pelvis"},
            {"unitree_h1_2", "pelvis"},
            {"booster_t1", "Waist"},
            {"booster_t1_29dof", "Waist"},
            {"stanford_toddy", "waist_link"},
            {"fourier_n1", "base_link"},
            {"engineai_pm01", "LINK_BASE"},
            {"kuavo_s45", "base_link"},
            {"hightorque_hi", "base_link"},
            {"galaxea_r1pro", "torso_link4"},
            {"berkeley_humanoid_lite", "imu_2"},
            {"booster_k1", "Trunk"},
            {"pnd_adam_lite", "pelvis"},
            {"tienkung", "Base_link"},
            {"pal_talos", "base_link"},
            {"fourier_gr3", "base_link"},
        };
        return kMap;
    }

    const std::unordered_map<std::string, double>& viewerCamDistanceMap() {
        static const std::unordered_map<std::string, double> kMap = {
            {"unitree_g1", 2.0},
            {"unitree_g1_with_hands", 2.0},
            {"unitree_h1", 3.0},
            {"unitree_h1_2", 3.0},
            {"booster_t1", 2.0},
            {"booster_t1_29dof", 2.0},
            {"stanford_toddy", 1.0},
            {"fourier_n1", 2.0},
            {"engineai_pm01", 2.0},
            {"kuavo_s45", 3.0},
            {"hightorque_hi", 2.0},
            {"galaxea_r1pro", 3.0},
            {"berkeley_humanoid_lite", 2.0},
            {"booster_k1", 2.0},
            {"pnd_adam_lite", 3.0},
            {"tienkung", 3.0},
            {"pal_talos", 3.0},
            {"fourier_gr3", 2.0},
        };
        return kMap;
    }

    std::string resolveCameraBodyName(const std::string& robot, const std::string& fallback) {
        const auto& map = robotBaseMap();
        auto it         = map.find(robot);
        if (it != map.end()) {
            return it->second;
        }
        return fallback;
    }

    double resolveCameraDistance(const std::string& robot, double fallback) {
        const auto& map = viewerCamDistanceMap();
        auto it         = map.find(robot);
        if (it != map.end()) {
            return it->second;
        }
        return fallback;
    }

    void drawAxesAtPose(const double pos[3], const double xmat[9], mjvScene* scene, double scale) {
        static const std::array<std::array<float, 4>, 3> kColors = {
            std::array<float, 4>{1.0f, 0.0f, 0.0f, 1.0f},
            std::array<float, 4>{0.0f, 1.0f, 0.0f, 1.0f},
            std::array<float, 4>{0.0f, 0.0f, 1.0f, 1.0f},
        };

        for (int axis = 0; axis < 3; ++axis) {
            if (scene->ngeom >= scene->maxgeom) {
                return;
            }
            mjvGeom* geom  = &scene->geoms[scene->ngeom];
            mjtNum from[3] = {pos[0], pos[1], pos[2]};
            mjtNum to[3]   = {
                pos[0] + scale * xmat[axis + 0],
                pos[1] + scale * xmat[axis + 3],
                pos[2] + scale * xmat[axis + 6],
            };
            mjtNum mjSize[3] = {0.01, 0.01, 0.01};
            float rgba[4]    = {kColors[axis][0], kColors[axis][1], kColors[axis][2], kColors[axis][3]};
            mjv_initGeom(geom, mjGEOM_ARROW, mjSize, from, xmat, rgba);
            mjv_connector(geom, mjGEOM_ARROW, 0.005, from, to);
            scene->ngeom += 1;
        }
    }

    bool loadConfig(const std::string& path, VisualizerConfig* cfg, std::string* error) {
        try {
            YAML::Node root = YAML::LoadFile(path);
            if (root["node_name"])
                cfg->nodeName = root["node_name"].as<std::string>();
            if (root["topic_name"])
                cfg->topicName = root["topic_name"].as<std::string>();
            if (root["mujoco_model_xml"]) {
                cfg->mujocoModelXml = root["mujoco_model_xml"].as<std::string>();
            }
            if (root["robot_name"])
                cfg->robotName = root["robot_name"].as<std::string>();
            if (root["camera_body_name"])
                cfg->cameraBodyName = root["camera_body_name"].as<std::string>();
            if (root["camera_distance"])
                cfg->cameraDistance = root["camera_distance"].as<double>();
            if (root["camera_azimuth"])
                cfg->cameraAzimuth = root["camera_azimuth"].as<double>();
            if (root["transparent_robot"])
                cfg->transparentRobot = root["transparent_robot"].as<int>();
            if (root["show_body_overlay"])
                cfg->showBodyOverlay = root["show_body_overlay"].as<bool>();
            if (root["body_overlay_scale"])
                cfg->bodyOverlayScale = root["body_overlay_scale"].as<double>();
            if (root["overlay_body_names"])
                cfg->overlayBodyNames = root["overlay_body_names"].as<std::vector<std::string>>();
            if (root["window_width"])
                cfg->width = root["window_width"].as<int>();
            if (root["window_height"])
                cfg->height = root["window_height"].as<int>();
            if (root["render_hz"])
                cfg->renderHz = root["render_hz"].as<int>();
            if (root["print_stat"])
                cfg->printStat = root["print_stat"].as<bool>();
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
            if (messageCount != nullptr)
                *messageCount = messageCount_;
            if (lastSeq != nullptr)
                *lastSeq = lastSeq_;
            return jointPosition_;
        }

       private:
        mutable std::mutex mutex_;
        std::unordered_map<std::string, double> jointPosition_;
        int64_t messageCount_ = 0;
        uint64_t lastSeq_     = 0;
    };

}  // namespace puppet::tools::retargeting_mujoco_visualizer_internal

int main(int argc, char** argv) {
    using namespace puppet::tools::retargeting_mujoco_visualizer_internal;
    std::string configPath = "config/tools/retargeting_mujoco_visualizer.yaml";
    if (argc > 1) {
        configPath = argv[1];
    }

    VisualizerConfig cfg;
    std::string error;
    if (!loadConfig(configPath, &cfg, &error)) {
        std::cerr << "[retargeting_mujoco_visualizer] load config failed: " << error << std::endl;
        return 1;
    }

    char modelError[1024] = {0};
    mjModel* modelRaw     = mj_loadXML(cfg.mujocoModelXml.c_str(), nullptr, modelError, sizeof(modelError));
    if (modelRaw == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] load model failed: " << modelError << std::endl;
        return 2;
    }
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(modelRaw, &mj_deleteModel);

    mjData* dataRaw = mj_makeData(model.get());
    if (dataRaw == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] make data failed" << std::endl;
        return 3;
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(dataRaw, &mj_deleteData);

    if (!glfwInit()) {
        std::cerr << "[retargeting_mujoco_visualizer] glfw init failed" << std::endl;
        return 4;
    }

    GLFWwindow* window = glfwCreateWindow(cfg.width, cfg.height, "PUPPET Retargeting Viewer", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] create window failed" << std::endl;
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

    camera.type                 = mjCAMERA_TRACKING;
    const double cameraDistance = resolveCameraDistance(cfg.robotName, cfg.cameraDistance);
    camera.distance             = cameraDistance;
    camera.azimuth              = cfg.cameraAzimuth;
    camera.elevation            = -10.0;

    const std::string resolvedCameraBodyName = resolveCameraBodyName(cfg.robotName, cfg.cameraBodyName);
    int cameraBodyId                         = mj_name2id(model.get(), mjOBJ_BODY, resolvedCameraBodyName.c_str());
    if (cameraBodyId < 0) {
        cameraBodyId = mj_name2id(model.get(), mjOBJ_BODY, cfg.cameraBodyName.c_str());
    }
    if (cameraBodyId >= 0) {
        camera.trackbodyid = cameraBodyId;
        camera.lookat[0]   = data->xpos[3 * cameraBodyId + 0];
        camera.lookat[1]   = data->xpos[3 * cameraBodyId + 1];
        camera.lookat[2]   = data->xpos[3 * cameraBodyId + 2];
    } else {
        std::cerr << "[retargeting_mujoco_visualizer] warning: camera body not found. resolved=" << resolvedCameraBodyName
                  << ", fallback=" << cfg.cameraBodyName << std::endl;
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

    ControlIntentCache cache;

    galbot::embosa::EmbosaInit();
    auto node = galbot::embosa::CreateNode(cfg.nodeName);
    if (node == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] create embosa node failed" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 6;
    }

    auto callback = [&](const std::shared_ptr<::puppet::puppet_proto::ControlIntent>& msg, const void*) {
        if (msg == nullptr) {
            return;
        }
        cache.update(*msg);
    };

    auto reader = node->CreateReader<::puppet::puppet_proto::ControlIntent>(cfg.topicName, callback);
    if (reader == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] create reader failed" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 7;
    }

    const int renderHz = cfg.renderHz > 0 ? cfg.renderHz : 60;
    const auto sleepDt = std::chrono::milliseconds(1000 / renderHz);

    while (!glfwWindowShouldClose(window) && galbot::embosa::OK()) {
        int64_t messageCount = 0;
        uint64_t lastSeq     = 0;
        const auto jointMap  = cache.snapshot(&messageCount, &lastSeq);
        int mappedCount      = 0;

        for (const auto& kv : jointMap) {
            const auto it = jointQposAdr.find(kv.first);
            if (it == jointQposAdr.end()) {
                continue;
            }
            data->qpos[it->second] = kv.second;
            ++mappedCount;
        }

        mj_forward(model.get(), data.get());
        if (cameraBodyId >= 0) {
            camera.type        = mjCAMERA_TRACKING;
            camera.trackbodyid = cameraBodyId;
            camera.lookat[0]   = data->xpos[3 * cameraBodyId + 0];
            camera.lookat[1]   = data->xpos[3 * cameraBodyId + 1];
            camera.lookat[2]   = data->xpos[3 * cameraBodyId + 2];
            camera.distance    = cameraDistance;
            camera.azimuth     = cfg.cameraAzimuth;
            camera.elevation   = -10.0;
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(model.get(), data.get(), &option, nullptr, &camera, mjCAT_ALL, &scene);
        if (cfg.showBodyOverlay) {
            for (const auto& bodyName : cfg.overlayBodyNames) {
                const int bodyId = mj_name2id(model.get(), mjOBJ_BODY, bodyName.c_str());
                if (bodyId < 0) {
                    continue;
                }
                const double* pos  = &data->xpos[3 * bodyId];
                const double* xmat = &data->xmat[9 * bodyId];
                drawAxesAtPose(pos, xmat, &scene, cfg.bodyOverlayScale);
            }
        }
        mjr_render(viewport, &scene, &context);

        if (cfg.printStat) {
            static int64_t lastPrintCount = -1;
            if (messageCount != lastPrintCount) {
                lastPrintCount = messageCount;
                std::cout << "[retargeting_mujoco_visualizer] msg_count=" << messageCount << " last_seq=" << lastSeq
                          << " cache_joints=" << jointMap.size() << " mapped_joints=" << mappedCount << std::endl;
            }
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(sleepDt);
    }

    galbot::embosa::WaitForShutdown();
    galbot::embosa::Clear();

    mjr_freeContext(&context);
    mjv_freeScene(&scene);
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
