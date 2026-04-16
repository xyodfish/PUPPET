#include "gmr/retarget/retargeter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "gmr/solver/qp_solver.h"
#include "retargeter_internal_utils.h"

namespace gmr {

    struct PinLegacyTaskRuntime {
        pinocchio::FrameIndex frameId = 0;
        bool useJointPose             = false;
        pinocchio::JointIndex jointId = 0;
        std::string humanBodyName;
        double posWeight             = 0.0;
        double rotWeight             = 0.0;
        Eigen::Vector3d posOffset    = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotOffset = Eigen::Quaterniond::Identity();

        Eigen::Vector3d targetPos    = Eigen::Vector3d::Zero();
        Eigen::Quaterniond targetRot = Eigen::Quaterniond::Identity();
    };

    std::string readTextFileLegacy(const std::filesystem::path& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            throw std::runtime_error("Failed to open file: " + path.string());
        }
        std::ostringstream oss;
        oss << ifs.rdbuf();
        return oss.str();
    }

    void writeTextFileLegacy(const std::filesystem::path& path, const std::string& text) {
        std::ofstream ofs(path);
        if (!ofs.is_open()) {
            throw std::runtime_error("Failed to open file for writing: " + path.string());
        }
        ofs << text;
    }

    std::size_t findMatchingTagEndLegacy(const std::string& xml, const std::string& tag, std::size_t openPos) {
        const std::string openTag  = "<" + tag;
        const std::string closeTag = "</" + tag + ">";

        int depth          = 1;
        std::size_t cursor = openPos + openTag.size();
        while (depth > 0) {
            const std::size_t nextOpen  = xml.find(openTag, cursor);
            const std::size_t nextClose = xml.find(closeTag, cursor);
            if (nextClose == std::string::npos) {
                throw std::runtime_error("Malformed XML: missing closing tag for <" + tag + ">.");
            }

            if (nextOpen != std::string::npos && nextOpen < nextClose) {
                depth += 1;
                cursor = nextOpen + openTag.size();
            } else {
                depth -= 1;
                cursor = nextClose + closeTag.size();
            }
        }
        return cursor;
    }

    std::string stripRepeatedTopLevelTagLegacy(const std::string& xml, const std::string& tag) {
        const std::string openTag = "<" + tag;

        std::string out             = xml;
        const std::size_t firstOpen = out.find(openTag);
        if (firstOpen == std::string::npos) {
            return out;
        }

        std::size_t firstEnd = findMatchingTagEndLegacy(out, tag, firstOpen);
        while (true) {
            const std::size_t nextOpen = out.find(openTag, firstEnd);
            if (nextOpen == std::string::npos) {
                break;
            }
            const std::size_t nextEnd = findMatchingTagEndLegacy(out, tag, nextOpen);
            out.erase(nextOpen, nextEnd - nextOpen);
        }

        return out;
    }

    std::string sanitizeMjcfForPinocchioLegacy(const std::filesystem::path& path) {
        std::string xml = readTextFileLegacy(path);
        xml             = stripRepeatedTopLevelTagLegacy(xml, "asset");
        xml             = stripRepeatedTopLevelTagLegacy(xml, "worldbody");
        return xml;
    }

    void buildSanitizedMjcfModelLegacy(const std::filesystem::path& path, pinocchio::Model* model) {
        const std::string sanitized         = sanitizeMjcfForPinocchioLegacy(path);
        const std::filesystem::path tmpPath = path.parent_path() / ".pinocchio_sanitized_tmp.xml";
        writeTextFileLegacy(tmpPath, sanitized);
        try {
            pinocchio::mjcf::buildModel(tmpPath.string(), *model);
        } catch (...) {
            std::error_code ec;
            std::filesystem::remove(tmpPath, ec);
            throw;
        }
        std::error_code ec;
        std::filesystem::remove(tmpPath, ec);
    }

    std::optional<pinocchio::FrameIndex> findFrameByNameAndTypeLegacy(const pinocchio::Model& model, const std::string& name,
                                                                      pinocchio::FrameType type) {
        for (pinocchio::FrameIndex i = 0; i < model.nframes; ++i) {
            const auto& frame = model.frames[i];
            if (frame.name == name && frame.type == type) {
                return i;
            }
        }
        return std::nullopt;
    }

    std::pair<pinocchio::FrameIndex, pinocchio::FrameType> resolveTaskFrameIdLegacy(const pinocchio::Model& model,
                                                                                    const std::string& name) {
        static const std::array<pinocchio::FrameType, 4> kPriority = {pinocchio::BODY, pinocchio::OP_FRAME, pinocchio::FIXED_JOINT,
                                                                      pinocchio::JOINT};
        for (pinocchio::FrameType type : kPriority) {
            std::optional<pinocchio::FrameIndex> frameId = findFrameByNameAndTypeLegacy(model, name, type);
            if (frameId.has_value()) {
                return {*frameId, type};
            }
        }
        throw std::runtime_error("Frame not found in Pinocchio model: " + name);
    }

    struct PinocchioLegacyRetargetBackend::Impl {
        pinocchio::Model model;
        std::unique_ptr<pinocchio::Data> data;

        IkConfig ikConfig;
        RetargetOptions options;

        std::vector<PinLegacyTaskRuntime> tasks1;
        std::vector<PinLegacyTaskRuntime> tasks2;
        std::unordered_map<std::string, Eigen::Vector3d> table1PosOffsets;
        std::unordered_map<std::string, Eigen::Quaterniond> table1RotOffsets;

        bool hasRootFreeFlyer = false;
        std::vector<ScalarJointCoordinate> scalarJointCoordinates;
        Eigen::VectorXd qpos;
        Eigen::VectorXd qposPin;
        Eigen::VectorXd qvel;

        Impl(const std::filesystem::path& robotModelPath, IkConfig ikConfigIn, RetargetOptions optionsIn)
            : ikConfig(std::move(ikConfigIn)), options(std::move(optionsIn)) {
            const std::string extension = robotModelPath.extension().string();
            try {
                if (extension == ".urdf") {
                    pinocchio::urdf::buildModel(robotModelPath.string(), pinocchio::JointModelFreeFlyer(), model);
                } else if (extension == ".xml" || extension == ".mjcf") {
                    buildSanitizedMjcfModelLegacy(robotModelPath, &model);
                } else {
                    throw std::runtime_error("Unsupported robot model extension for Pinocchio: " + extension);
                }
            } catch (const std::exception& e) {
                throw std::runtime_error("Failed to load robot model for Pinocchio: " + robotModelPath.string() + " error=" + e.what());
            }

            if (model.nq <= 0 || model.nv <= 0) {
                throw std::runtime_error("Pinocchio model has invalid nq/nv.");
            }

            data             = std::make_unique<pinocchio::Data>(model);
            hasRootFreeFlyer = (model.njoints > 1 && model.joints[1].nq() == 7 && model.joints[1].nv() == 6);

            for (pinocchio::JointIndex jointId = 1; jointId < model.njoints; ++jointId) {
                const auto& jointModel = model.joints[jointId];
                if (jointModel.nq() == 1 && jointModel.nv() == 1) {
                    scalarJointCoordinates.push_back(ScalarJointCoordinate{jointModel.idx_q(), jointModel.idx_v(), model.names[jointId]});
                }
            }

            qposPin = pinocchio::neutral(model);
            qvel    = Eigen::VectorXd::Zero(model.nv);
            qpos    = pinocchioToMujocoQpos(qposPin);

            auto buildTasks = [this](const std::vector<IkTaskEntry>& src, std::vector<PinLegacyTaskRuntime>* dst) {
                dst->clear();
                dst->reserve(src.size());
                for (const auto& entry : src) {
                    PinLegacyTaskRuntime task;
                    auto [frameId, frameType] = resolveTaskFrameIdLegacy(model, entry.robotBodyName);
                    (void)frameType;
                    task.frameId = frameId;
                    if (hasRootFreeFlyer && entry.robotBodyName == ikConfig.robotRootName) {
                        task.useJointPose = true;
                        task.jointId      = 1;
                    }
                    task.humanBodyName = entry.humanBodyName;
                    task.posWeight     = entry.posWeight;
                    task.rotWeight     = entry.rotWeight;
                    task.posOffset     = entry.posOffset;
                    task.rotOffset     = entry.rotOffset;
                    dst->push_back(std::move(task));
                }
            };

            buildTasks(ikConfig.tasksTable1, &tasks1);
            buildTasks(ikConfig.tasksTable2, &tasks2);

            for (const auto& task : tasks1) {
                table1PosOffsets[task.humanBodyName] = task.posOffset - Eigen::Vector3d(0.0, 0.0, ikConfig.groundHeight);
                table1RotOffsets[task.humanBodyName] = task.rotOffset;
            }

            syncDataFromQpos();
        }

        Eigen::VectorXd pinocchioToMujocoQpos(const Eigen::VectorXd& qPinIn) const {
            Eigen::VectorXd qMj = qPinIn;
            if (hasRootFreeFlyer && qMj.size() >= 7) {
                qMj[3] = qPinIn[6];
                qMj[4] = qPinIn[3];
                qMj[5] = qPinIn[4];
                qMj[6] = qPinIn[5];
            }
            return qMj;
        }

        Eigen::VectorXd mujocoToPinocchioQpos(const Eigen::VectorXd& qMjIn) const {
            Eigen::VectorXd qPin = qMjIn;
            if (hasRootFreeFlyer && qPin.size() >= 7) {
                qPin[3] = qMjIn[4];
                qPin[4] = qMjIn[5];
                qPin[5] = qMjIn[6];
                qPin[6] = qMjIn[3];
            }
            return qPin;
        }

        void syncDataFromQpos() {
            qpos = pinocchioToMujocoQpos(qposPin);
            pinocchio::forwardKinematics(model, *data, qposPin, qvel);
            pinocchio::updateFramePlacements(model, *data);
        }

        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround) const {
            return retarget_internal::scaleAndOffsetHumanFrameImpl(humanFrame, ikConfig, table1PosOffsets, table1RotOffsets,
                                                                   offsetToGround);
        }

        void updateTaskTargets(const HumanFrame& frame) {
            auto fill = [&frame](std::vector<PinLegacyTaskRuntime>* tasks) {
                for (auto& task : *tasks) {
                    auto it = frame.find(task.humanBodyName);
                    if (it == frame.end()) {
                        continue;
                    }
                    task.targetPos = it->second.position;
                    task.targetRot = it->second.orientation;
                }
            };

            fill(&tasks1);
            fill(&tasks2);
        }

        double computeTaskError(const std::vector<PinLegacyTaskRuntime>& tasks) const {
            double sqErr = 0.0;
            for (const auto& task : tasks) {
                Eigen::Vector3d currPos    = Eigen::Vector3d::Zero();
                Eigen::Quaterniond currRot = Eigen::Quaterniond::Identity();
                if (task.useJointPose) {
                    if (task.jointId >= model.njoints) {
                        continue;
                    }
                    const pinocchio::SE3& pose = data->oMi[task.jointId];
                    currPos                    = pose.translation();
                    currRot                    = Eigen::Quaterniond(pose.rotation());
                } else {
                    if (task.frameId >= model.nframes) {
                        continue;
                    }
                    const pinocchio::SE3& pose = data->oMf[task.frameId];
                    currPos                    = pose.translation();
                    currRot                    = Eigen::Quaterniond(pose.rotation());
                }
                currRot.normalize();

                const Eigen::Vector3d posErr = task.targetPos - currPos;
                const Eigen::Vector3d rotErr = retarget_internal::computeOrientationErrorWorld(currRot, task.targetRot);
                sqErr += posErr.squaredNorm() + rotErr.squaredNorm();
            }
            return std::sqrt(sqErr);
        }

        void solveTaskSet(const std::vector<PinLegacyTaskRuntime>& tasks) {
            if (tasks.empty()) {
                return;
            }

            const int nv    = model.nv;
            const double dt = options.integrationTimestep;
            if (dt <= 1e-12) {
                throw std::runtime_error("integrationTimestep must be positive.");
            }
            const double invDt = 1.0 / dt;

            double currError = computeTaskError(tasks);
            solver::QPSolver solver;

            for (int iter = 0; iter < options.maxIterations; ++iter) {
                solver::QPData qp;
                qp.reset(nv, nv);

                qp.CI.setIdentity();
                qp.ciLb.setConstant(-1e9);
                qp.ciUb.setConstant(1e9);

                if (options.useVelocityLimit) {
                    qp.ciLb.setConstant(-options.velocityLimit);
                    qp.ciUb.setConstant(options.velocityLimit);
                }

                for (pinocchio::JointIndex jointId = 1; jointId < model.njoints; ++jointId) {
                    const auto& jointModel = model.joints[jointId];
                    if (jointModel.nq() != 1 || jointModel.nv() != 1) {
                        continue;
                    }

                    const int qadr    = jointModel.idx_q();
                    const int vadr    = jointModel.idx_v();
                    const double qmin = model.lowerPositionLimit[qadr];
                    const double qmax = model.upperPositionLimit[qadr];
                    if (std::isfinite(qmin) && std::isfinite(qmax)) {
                        qp.ciLb[vadr] = std::max(qp.ciLb[vadr], (qmin - qposPin[qadr]) / dt);
                        qp.ciUb[vadr] = std::min(qp.ciUb[vadr], (qmax - qposPin[qadr]) / dt);
                    }
                }

                Eigen::Matrix<double, 6, Eigen::Dynamic> frameJacobian(6, nv);
                pinocchio::computeJointJacobians(model, *data, qposPin);

                for (const auto& task : tasks) {
                    Eigen::Vector3d currPos    = Eigen::Vector3d::Zero();
                    Eigen::Quaterniond currRot = Eigen::Quaterniond::Identity();

                    if (task.useJointPose) {
                        if (task.jointId >= model.njoints) {
                            continue;
                        }
                        frameJacobian = pinocchio::getJointJacobian(model, *data, task.jointId, pinocchio::LOCAL_WORLD_ALIGNED);
                        const pinocchio::SE3& pose = data->oMi[task.jointId];
                        currPos                    = pose.translation();
                        currRot                    = Eigen::Quaterniond(pose.rotation());
                    } else {
                        if (task.frameId >= model.nframes) {
                            continue;
                        }
                        frameJacobian.setZero();
                        pinocchio::computeFrameJacobian(model, *data, qposPin, task.frameId, pinocchio::LOCAL_WORLD_ALIGNED, frameJacobian);
                        const pinocchio::SE3& pose = data->oMf[task.frameId];
                        currPos                    = pose.translation();
                        currRot                    = Eigen::Quaterniond(pose.rotation());
                    }

                    currRot.normalize();

                    const auto Jp = frameJacobian.topRows<3>();
                    const auto Jr = frameJacobian.bottomRows<3>();

                    const Eigen::Vector3d posErr       = task.targetPos - currPos;
                    const Eigen::Vector3d rotErr       = retarget_internal::computeOrientationErrorWorld(currRot, task.targetRot);
                    const Eigen::Vector3d posTargetVel = posErr * invDt;
                    const Eigen::Vector3d rotTargetVel = rotErr * invDt;

                    if (task.posWeight > 0.0) {
                        qp.H.noalias() += task.posWeight * (Jp.transpose() * Jp);
                        qp.g.noalias() += -task.posWeight * (Jp.transpose() * posTargetVel);
                    }
                    if (task.rotWeight > 0.0) {
                        qp.H.noalias() += task.rotWeight * (Jr.transpose() * Jr);
                        qp.g.noalias() += -task.rotWeight * (Jr.transpose() * rotTargetVel);
                    }
                }

                qp.H.diagonal().array() += options.damping;

                const solver::QPOutput& out = solver.solve(qp);
                if (out.status != solver::QPStatus::kOptimal) {
                    throw std::runtime_error("QP solver failed while retargeting.");
                }

                qvel    = out.x;
                qposPin = pinocchio::integrate(model, qposPin, qvel * dt);
                syncDataFromQpos();

                const double nextError = computeTaskError(tasks);
                if (currError - nextError <= options.progressThreshold) {
                    break;
                }
                currError = nextError;
            }
        }
    };

    PinocchioLegacyRetargetBackend::PinocchioLegacyRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig,
                                                                   RetargetOptions options)
        : impl_(std::make_unique<Impl>(robotModelPath, std::move(ikConfig), std::move(options))) {}

    PinocchioLegacyRetargetBackend::~PinocchioLegacyRetargetBackend() = default;

    Eigen::VectorXd PinocchioLegacyRetargetBackend::retargetFrame(const HumanFrame& humanFrame, bool offsetToGround) {
        HumanFrame prepared = impl_->prepareHumanFrame(humanFrame, offsetToGround);
        impl_->updateTaskTargets(prepared);
        if (impl_->ikConfig.useTable1) {
            impl_->solveTaskSet(impl_->tasks1);
        }
        if (impl_->ikConfig.useTable2) {
            impl_->solveTaskSet(impl_->tasks2);
        }
        return impl_->qpos;
    }

    HumanFrame PinocchioLegacyRetargetBackend::prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround) const {
        return impl_->prepareHumanFrame(humanFrame, offsetToGround);
    }

    void PinocchioLegacyRetargetBackend::setQpos(const Eigen::VectorXd& qpos) {
        if (qpos.size() != impl_->qpos.size()) {
            throw std::runtime_error("setQpos size mismatch.");
        }
        impl_->qpos    = qpos;
        impl_->qposPin = impl_->mujocoToPinocchioQpos(impl_->qpos);
        impl_->qvel.setZero();
        impl_->syncDataFromQpos();
    }

    const Eigen::VectorXd& PinocchioLegacyRetargetBackend::currentQpos() const { return impl_->qpos; }

    bool PinocchioLegacyRetargetBackend::hasRootFreeFlyer() const { return impl_->hasRootFreeFlyer; }

    const std::vector<ScalarJointCoordinate>& PinocchioLegacyRetargetBackend::scalarJointCoordinates() const {
        return impl_->scalarJointCoordinates;
    }

}  // namespace gmr
