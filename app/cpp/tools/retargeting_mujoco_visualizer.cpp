#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include "embosa.hpp"
#include "gmr/retarget/ik_config.h"
#include "gmr/retarget/retargeter.h"
#include "puppet/control_intent.pb.h"
#include "puppet/primitive_frame.pb.h"

namespace puppet::tools::retargeting_mujoco_visualizer_internal {
    namespace visualizer_detail {
        constexpr const char* kRootPosXKey  = "__root_pos_x";
        constexpr const char* kRootPosYKey  = "__root_pos_y";
        constexpr const char* kRootPosZKey  = "__root_pos_z";
        constexpr const char* kRootQuatWKey = "__root_quat_w";
        constexpr const char* kRootQuatXKey = "__root_quat_x";
        constexpr const char* kRootQuatYKey = "__root_quat_y";
        constexpr const char* kRootQuatZKey = "__root_quat_z";
    }  // namespace visualizer_detail

    struct VisualizerConfig {
        std::string robotQposSource     = "control_intent";  // control_intent | local_retarget
        std::string nodeName            = "puppet_retargeting_mujoco_visualizer";
        std::string topicName           = "puppet_demo/control_intent";
        std::string humanFrameTopicName = "puppet_demo/primitive_frame";
        std::string mujocoModelXml;
        std::string robotName                  = "unitree_g1";
        std::string cameraBodyName             = "pelvis";
        double cameraDistance                  = 2.5;
        double cameraAzimuth                   = 0.0;
        bool hasCameraAzimuth                  = false;
        int transparentRobot                   = 0;
        bool showBodyOverlay                   = true;
        double bodyOverlayScale                = 0.1;
        bool showHumanOverlay                  = true;
        double humanOverlayScale               = 0.1;
        bool humanOverlayUsePreparedFrame      = true;
        std::string humanOverlayBackend        = "mujoco_se3";
        std::string humanOverlayRobotModelPath = "assets/unitree_g1/g1_mocap_29dof.xml";
        std::string humanOverlayIkConfigPath = "/data/open_src_code/GMR_custom/general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json";
        double humanOverlayActualHumanHeight = 1.75;
        bool humanOverlayOffsetToGround      = false;
        std::string localRetargetBackend     = "mujoco_se3";
        std::string localRetargetRobotModelPath = "assets/unitree_g1/g1_mocap_29dof.xml";
        std::string localRetargetIkConfigPath =
            "/data/open_src_code/GMR_custom/general_motion_retargeting/ik_configs/bvh_lafan1_to_g1.json";
        double localRetargetActualHumanHeight     = 1.75;
        double localRetargetDamping               = 0.5;
        int localRetargetMaxIterations            = 20;
        bool localRetargetUseVelocityLimit        = false;
        double localRetargetIntegrationTimestep   = 0.01;
        double localRetargetProgressThreshold     = 0.001;
        bool localRetargetOffsetToGround          = false;
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
            if (root["robot_qpos_source"])
                cfg->robotQposSource = root["robot_qpos_source"].as<std::string>();
            if (root["node_name"])
                cfg->nodeName = root["node_name"].as<std::string>();
            if (root["topic_name"])
                cfg->topicName = root["topic_name"].as<std::string>();
            if (root["human_frame_topic_name"])
                cfg->humanFrameTopicName = root["human_frame_topic_name"].as<std::string>();
            if (root["mujoco_model_xml"]) {
                cfg->mujocoModelXml = root["mujoco_model_xml"].as<std::string>();
            }
            if (root["robot_name"])
                cfg->robotName = root["robot_name"].as<std::string>();
            if (root["camera_body_name"])
                cfg->cameraBodyName = root["camera_body_name"].as<std::string>();
            if (root["camera_distance"])
                cfg->cameraDistance = root["camera_distance"].as<double>();
            if (root["camera_azimuth"]) {
                cfg->cameraAzimuth    = root["camera_azimuth"].as<double>();
                cfg->hasCameraAzimuth = true;
            }
            if (root["transparent_robot"])
                cfg->transparentRobot = root["transparent_robot"].as<int>();
            if (root["show_body_overlay"])
                cfg->showBodyOverlay = root["show_body_overlay"].as<bool>();
            if (root["body_overlay_scale"])
                cfg->bodyOverlayScale = root["body_overlay_scale"].as<double>();
            if (root["show_human_overlay"])
                cfg->showHumanOverlay = root["show_human_overlay"].as<bool>();
            if (root["human_overlay_scale"])
                cfg->humanOverlayScale = root["human_overlay_scale"].as<double>();
            if (root["human_overlay_use_prepared_frame"])
                cfg->humanOverlayUsePreparedFrame = root["human_overlay_use_prepared_frame"].as<bool>();
            if (root["human_overlay_backend"])
                cfg->humanOverlayBackend = root["human_overlay_backend"].as<std::string>();
            if (root["human_overlay_robot_model_path"])
                cfg->humanOverlayRobotModelPath = root["human_overlay_robot_model_path"].as<std::string>();
            if (root["human_overlay_ik_config_path"])
                cfg->humanOverlayIkConfigPath = root["human_overlay_ik_config_path"].as<std::string>();
            if (root["human_overlay_actual_human_height"])
                cfg->humanOverlayActualHumanHeight = root["human_overlay_actual_human_height"].as<double>();
            if (root["human_overlay_offset_to_ground"])
                cfg->humanOverlayOffsetToGround = root["human_overlay_offset_to_ground"].as<bool>();
            if (root["local_retarget_backend"])
                cfg->localRetargetBackend = root["local_retarget_backend"].as<std::string>();
            if (root["local_retarget_robot_model_path"])
                cfg->localRetargetRobotModelPath = root["local_retarget_robot_model_path"].as<std::string>();
            if (root["local_retarget_ik_config_path"])
                cfg->localRetargetIkConfigPath = root["local_retarget_ik_config_path"].as<std::string>();
            if (root["local_retarget_actual_human_height"])
                cfg->localRetargetActualHumanHeight = root["local_retarget_actual_human_height"].as<double>();
            if (root["local_retarget_damping"])
                cfg->localRetargetDamping = root["local_retarget_damping"].as<double>();
            if (root["local_retarget_max_iterations"])
                cfg->localRetargetMaxIterations = root["local_retarget_max_iterations"].as<int>();
            if (root["local_retarget_use_velocity_limit"])
                cfg->localRetargetUseVelocityLimit = root["local_retarget_use_velocity_limit"].as<bool>();
            if (root["local_retarget_integration_timestep"])
                cfg->localRetargetIntegrationTimestep = root["local_retarget_integration_timestep"].as<double>();
            if (root["local_retarget_progress_threshold"])
                cfg->localRetargetProgressThreshold = root["local_retarget_progress_threshold"].as<double>();
            if (root["local_retarget_offset_to_ground"])
                cfg->localRetargetOffsetToGround = root["local_retarget_offset_to_ground"].as<bool>();
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

    class HumanPoseCache {
       public:
        struct PoseAxes {
            std::array<double, 3> pos  = {0.0, 0.0, 0.0};
            std::array<double, 9> xmat = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        };

        class HumanOverlayPreprocessor {
           public:
            bool init(const VisualizerConfig& cfg, std::string* error) {
                if (!cfg.humanOverlayUsePreparedFrame) {
                    return true;
                }
                if (cfg.humanOverlayRobotModelPath.empty() || cfg.humanOverlayIkConfigPath.empty()) {
                    if (error != nullptr) {
                        *error = "human overlay preprocessor requires robot_model_path and ik_config_path";
                    }
                    return false;
                }
                const auto backend = gmr::parseRetargetBackend(cfg.humanOverlayBackend);
                gmr::RetargetOptions options;
                gmr::IkConfig ikConfig = gmr::loadIkConfig(cfg.humanOverlayIkConfigPath, cfg.humanOverlayActualHumanHeight);
                retargeter_            = gmr::createRetargeter(backend, cfg.humanOverlayRobotModelPath, std::move(ikConfig), options);
                offsetToGround_        = cfg.humanOverlayOffsetToGround;
                enabled_               = true;
                return true;
            }

            gmr::HumanFrame process(const gmr::HumanFrame& humanFrame) const {
                if (!enabled_ || retargeter_ == nullptr) {
                    return humanFrame;
                }
                return retargeter_->prepareHumanFrame(humanFrame, offsetToGround_);
            }

           private:
            bool enabled_        = false;
            bool offsetToGround_ = false;
            std::unique_ptr<gmr::Retargeter> retargeter_;
        };

        bool initPreprocessor(const VisualizerConfig& cfg, std::string* error) { return preprocessor_.init(cfg, error); }

        void update(const ::puppet::puppet_proto::PrimitiveFrame& msg) {
            gmr::HumanFrame humanFrame;
            for (const auto& pose : msg.poses()) {
                if (pose.meta().entity().empty()) {
                    continue;
                }
                gmr::HumanBodyState state;
                state.position    = Eigen::Vector3d(pose.pose().position().x(), pose.pose().position().y(), pose.pose().position().z());
                state.orientation = Eigen::Quaterniond(pose.pose().orientation().w(), pose.pose().orientation().x(),
                                                       pose.pose().orientation().y(), pose.pose().orientation().z());
                humanFrame[pose.meta().entity()] = state;
            }
            const gmr::HumanFrame preparedFrame = preprocessor_.process(humanFrame);

            std::vector<PoseAxes> localPoses;
            localPoses.reserve(preparedFrame.size());
            for (const auto& kv : preparedFrame) {
                const auto& body = kv.second;
                PoseAxes axes;
                axes.pos[0]                    = body.position.x();
                axes.pos[1]                    = body.position.y();
                axes.pos[2]                    = body.position.z();
                const Eigen::Matrix3d rotation = body.orientation.normalized().toRotationMatrix();
                axes.xmat                      = {
                    rotation(0, 0), rotation(0, 1), rotation(0, 2), rotation(1, 0), rotation(1, 1),
                    rotation(1, 2), rotation(2, 0), rotation(2, 1), rotation(2, 2),
                };
                localPoses.push_back(axes);
            }

            std::lock_guard<std::mutex> lk(mutex_);
            poses_ = std::move(localPoses);
        }

        std::vector<PoseAxes> snapshot() const {
            std::lock_guard<std::mutex> lk(mutex_);
            return poses_;
        }

       private:
        mutable std::mutex mutex_;
        std::vector<PoseAxes> poses_;
        HumanOverlayPreprocessor preprocessor_;
    };

    class LocalRetargetCache {
       public:
        bool init(const VisualizerConfig& cfg, std::string* error) {
            enabled_ = cfg.robotQposSource == "local_retarget";
            if (!enabled_) {
                return true;
            }
            if (cfg.localRetargetRobotModelPath.empty() || cfg.localRetargetIkConfigPath.empty()) {
                if (error != nullptr) {
                    *error = "local retarget requires robot model path and ik config path";
                }
                return false;
            }
            const auto backend = gmr::parseRetargetBackend(cfg.localRetargetBackend);
            gmr::RetargetOptions options;
            options.damping             = cfg.localRetargetDamping;
            options.maxIterations       = cfg.localRetargetMaxIterations;
            options.useVelocityLimit    = cfg.localRetargetUseVelocityLimit;
            options.integrationTimestep = cfg.localRetargetIntegrationTimestep;
            options.progressThreshold   = cfg.localRetargetProgressThreshold;
            gmr::IkConfig ikConfig      = gmr::loadIkConfig(cfg.localRetargetIkConfigPath, cfg.localRetargetActualHumanHeight);
            retargeter_                 = gmr::createRetargeter(backend, cfg.localRetargetRobotModelPath, std::move(ikConfig), options);
            offsetToGround_             = cfg.localRetargetOffsetToGround;
            return true;
        }

        void update(const ::puppet::puppet_proto::PrimitiveFrame& msg) {
            if (!enabled_ || retargeter_ == nullptr) {
                return;
            }
            gmr::HumanFrame humanFrame;
            for (const auto& pose : msg.poses()) {
                if (pose.meta().entity().empty()) {
                    continue;
                }
                gmr::HumanBodyState state;
                state.position    = Eigen::Vector3d(pose.pose().position().x(), pose.pose().position().y(), pose.pose().position().z());
                state.orientation = Eigen::Quaterniond(pose.pose().orientation().w(), pose.pose().orientation().x(),
                                                       pose.pose().orientation().y(), pose.pose().orientation().z());
                humanFrame[pose.meta().entity()] = state;
            }
            if (humanFrame.empty()) {
                return;
            }
            const Eigen::VectorXd qpos = retargeter_->retargetFrame(humanFrame, offsetToGround_);
            std::lock_guard<std::mutex> lk(mutex_);
            qpos_       = qpos;
            sequenceId_ = msg.sequence_id();
            hasQpos_    = true;
        }

        bool enabled() const { return enabled_; }

        bool snapshot(Eigen::VectorXd* qpos, uint64_t* sequenceId) const {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!hasQpos_) {
                return false;
            }
            if (qpos != nullptr) {
                *qpos = qpos_;
            }
            if (sequenceId != nullptr) {
                *sequenceId = sequenceId_;
            }
            return true;
        }

       private:
        bool enabled_        = false;
        bool offsetToGround_ = false;
        std::unique_ptr<gmr::Retargeter> retargeter_;
        mutable std::mutex mutex_;
        Eigen::VectorXd qpos_;
        uint64_t sequenceId_ = 0;
        bool hasQpos_        = false;
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
    camera.elevation            = -10.0;
    if (cfg.hasCameraAzimuth) {
        camera.azimuth = cfg.cameraAzimuth;
    }

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
    int freeJointQposAdr = -1;
    for (int j = 0; j < model->njnt; ++j) {
        if (model->jnt_type[j] == mjJNT_FREE) {
            freeJointQposAdr = model->jnt_qposadr[j];
            break;
        }
    }

    ControlIntentCache cache;
    HumanPoseCache humanPoseCache;
    LocalRetargetCache localRetargetCache;
    if (!humanPoseCache.initPreprocessor(cfg, &error)) {
        std::cerr << "[retargeting_mujoco_visualizer] init human overlay preprocessor failed: " << error << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 8;
    }
    if (!localRetargetCache.init(cfg, &error)) {
        std::cerr << "[retargeting_mujoco_visualizer] init local retarget failed: " << error << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 10;
    }

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
    auto humanCallback = [&](const std::shared_ptr<::puppet::puppet_proto::PrimitiveFrame>& msg, const void*) {
        if (msg == nullptr) {
            return;
        }
        humanPoseCache.update(*msg);
        localRetargetCache.update(*msg);
    };
    auto humanFrameReader = node->CreateReader<::puppet::puppet_proto::PrimitiveFrame>(cfg.humanFrameTopicName, humanCallback);
    if (humanFrameReader == nullptr) {
        std::cerr << "[retargeting_mujoco_visualizer] create human frame reader failed" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return 9;
    }

    const int renderHz = cfg.renderHz > 0 ? cfg.renderHz : 60;
    const auto sleepDt = std::chrono::milliseconds(1000 / renderHz);

    while (!glfwWindowShouldClose(window) && galbot::embosa::OK()) {
        int64_t messageCount      = 0;
        uint64_t lastSeq          = 0;
        const auto jointMap       = cache.snapshot(&messageCount, &lastSeq);
        uint64_t localRetargetSeq = 0;
        Eigen::VectorXd localQpos;
        const bool hasLocalQpos = localRetargetCache.snapshot(&localQpos, &localRetargetSeq);
        int mappedCount         = 0;

        // Keep the same rendering semantics as main_retarget_viewer:
        // start from model qpos0 every frame, then apply retargeted qpos.
        mju_copy(data->qpos, model->qpos0, model->nq);
        if (localRetargetCache.enabled() && hasLocalQpos && localQpos.size() == model->nq) {
            mju_copy(data->qpos, localQpos.data(), model->nq);
            mappedCount = model->nq;
            lastSeq     = localRetargetSeq;
        } else {
            for (const auto& kv : jointMap) {
                const auto it = jointQposAdr.find(kv.first);
                if (it == jointQposAdr.end()) {
                    continue;
                }
                data->qpos[it->second] = kv.second;
                ++mappedCount;
            }
            if (freeJointQposAdr >= 0) {
                const auto rootPosXIt  = jointMap.find(visualizer_detail::kRootPosXKey);
                const auto rootPosYIt  = jointMap.find(visualizer_detail::kRootPosYKey);
                const auto rootPosZIt  = jointMap.find(visualizer_detail::kRootPosZKey);
                const auto rootQuatWIt = jointMap.find(visualizer_detail::kRootQuatWKey);
                const auto rootQuatXIt = jointMap.find(visualizer_detail::kRootQuatXKey);
                const auto rootQuatYIt = jointMap.find(visualizer_detail::kRootQuatYKey);
                const auto rootQuatZIt = jointMap.find(visualizer_detail::kRootQuatZKey);
                const bool hasRootQpos = rootPosXIt != jointMap.end() && rootPosYIt != jointMap.end() && rootPosZIt != jointMap.end() &&
                                         rootQuatWIt != jointMap.end() && rootQuatXIt != jointMap.end() && rootQuatYIt != jointMap.end() &&
                                         rootQuatZIt != jointMap.end();
                if (hasRootQpos) {
                    data->qpos[freeJointQposAdr + 0] = rootPosXIt->second;
                    data->qpos[freeJointQposAdr + 1] = rootPosYIt->second;
                    data->qpos[freeJointQposAdr + 2] = rootPosZIt->second;
                    data->qpos[freeJointQposAdr + 3] = rootQuatWIt->second;
                    data->qpos[freeJointQposAdr + 4] = rootQuatXIt->second;
                    data->qpos[freeJointQposAdr + 5] = rootQuatYIt->second;
                    data->qpos[freeJointQposAdr + 6] = rootQuatZIt->second;
                    mappedCount += 7;
                }
            }
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
            camera.type        = mjCAMERA_TRACKING;
            camera.trackbodyid = cameraBodyId;
            camera.lookat[0]   = data->xpos[3 * cameraBodyId + 0];
            camera.lookat[1]   = data->xpos[3 * cameraBodyId + 1];
            camera.lookat[2]   = data->xpos[3 * cameraBodyId + 2];
            camera.distance    = cameraDistance;
            camera.elevation   = -10.0;
            if (cfg.hasCameraAzimuth) {
                camera.azimuth = cfg.cameraAzimuth;
            }
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(model.get(), data.get(), &option, nullptr, &camera, mjCAT_ALL, &scene);
        if (cfg.showHumanOverlay) {
            const auto humanPoses = humanPoseCache.snapshot();
            for (const auto& pose : humanPoses) {
                drawAxesAtPose(pose.pos.data(), pose.xmat.data(), &scene, cfg.humanOverlayScale);
            }
        }
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
                          << " source=" << (localRetargetCache.enabled() ? "local_retarget" : "control_intent")
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
