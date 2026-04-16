#include "gmr/retarget/retargeter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <mujoco/mujoco.h>

#include "gmr/solver/qp_solver.h"
#include "retargeter_internal_utils.h"

namespace gmr {

    struct MujocoLegacyTaskRuntime {
        int bodyId = -1;
        std::string humanBodyName;
        double posWeight             = 0.0;
        double rotWeight             = 0.0;
        Eigen::Vector3d posOffset    = Eigen::Vector3d::Zero();
        Eigen::Quaterniond rotOffset = Eigen::Quaterniond::Identity();

        Eigen::Vector3d targetPos    = Eigen::Vector3d::Zero();
        Eigen::Quaterniond targetRot = Eigen::Quaterniond::Identity();
    };

    struct MujocoLegacyRetargetBackend::Impl {
        struct ModelDeleter {
            void operator()(mjModel* p) const {
                if (p != nullptr) {
                    mj_deleteModel(p);
                }
            }
        };

        struct DataDeleter {
            void operator()(mjData* p) const {
                if (p != nullptr) {
                    mj_deleteData(p);
                }
            }
        };

        std::unique_ptr<mjModel, ModelDeleter> model;
        std::unique_ptr<mjData, DataDeleter> data;

        IkConfig ikConfig;
        RetargetOptions options;

        std::vector<MujocoLegacyTaskRuntime> tasks1;
        std::vector<MujocoLegacyTaskRuntime> tasks2;
        std::unordered_map<std::string, Eigen::Vector3d> table1PosOffsets;
        std::unordered_map<std::string, Eigen::Quaterniond> table1RotOffsets;

        bool hasRootFreeFlyer = false;
        std::vector<ScalarJointCoordinate> scalarJointCoordinates;
        Eigen::VectorXd qpos;
        Eigen::VectorXd qvel;

        Impl(const std::filesystem::path& robotModelPath, IkConfig ikConfigIn, RetargetOptions optionsIn)
            : ikConfig(std::move(ikConfigIn)), options(std::move(optionsIn)) {
            const std::string extension = retarget_internal::toLower(robotModelPath.extension().string());
            if (extension != ".xml" && extension != ".mjcf") {
                throw std::runtime_error("MuJoCo retarget backend requires XML/MJCF model: " + robotModelPath.string());
            }

            std::array<char, 1024> error{};
            mjModel* rawModel = mj_loadXML(robotModelPath.string().c_str(), nullptr, error.data(), error.size());
            if (rawModel == nullptr) {
                throw std::runtime_error("Failed to load MuJoCo model: " + robotModelPath.string() + " error=" + std::string(error.data()));
            }

            model.reset(rawModel);
            data.reset(mj_makeData(model.get()));
            if (!data) {
                throw std::runtime_error("Failed to allocate MuJoCo data.");
            }

            hasRootFreeFlyer = false;
            for (int j = 0; j < model->njnt; ++j) {
                if (model->jnt_type[j] == mjJNT_FREE) {
                    hasRootFreeFlyer = true;
                    break;
                }
            }

            for (int j = 0; j < model->njnt; ++j) {
                const int jointType = model->jnt_type[j];
                if (jointType == mjJNT_HINGE || jointType == mjJNT_SLIDE) {
                    const int qadr        = model->jnt_qposadr[j];
                    const int vadr        = model->jnt_dofadr[j];
                    const char* jointName = mj_id2name(model.get(), mjOBJ_JOINT, j);
                    scalarJointCoordinates.push_back(ScalarJointCoordinate{qadr, vadr, jointName == nullptr ? "" : jointName});
                }
            }

            std::sort(scalarJointCoordinates.begin(), scalarJointCoordinates.end(),
                      [](const ScalarJointCoordinate& a, const ScalarJointCoordinate& b) { return a.qIndex < b.qIndex; });

            auto buildTasks = [this](const std::vector<IkTaskEntry>& src, std::vector<MujocoLegacyTaskRuntime>* dst) {
                dst->clear();
                dst->reserve(src.size());
                for (const auto& entry : src) {
                    const int bodyId = mj_name2id(model.get(), mjOBJ_BODY, entry.robotBodyName.c_str());
                    if (bodyId < 0) {
                        throw std::runtime_error("Body not found in MuJoCo model: " + entry.robotBodyName);
                    }

                    MujocoLegacyTaskRuntime task;
                    task.bodyId        = bodyId;
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

            mju_copy(data->qpos, model->qpos0, model->nq);
            mju_zero(data->qvel, model->nv);
            mj_forward(model.get(), data.get());

            qpos = Eigen::Map<Eigen::VectorXd>(data->qpos, model->nq);
            qvel = Eigen::VectorXd::Zero(model->nv);
        }

        void syncQposFromData() { qpos = Eigen::Map<Eigen::VectorXd>(data->qpos, model->nq); }

        HumanFrame prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround) const {
            return retarget_internal::scaleAndOffsetHumanFrameImpl(humanFrame, ikConfig, table1PosOffsets, table1RotOffsets,
                                                                   offsetToGround);
        }

        void updateTaskTargets(const HumanFrame& frame) {
            auto fill = [&frame](std::vector<MujocoLegacyTaskRuntime>* tasks) {
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

        double computeTaskError(const std::vector<MujocoLegacyTaskRuntime>& tasks) const {
            double sqErr = 0.0;
            for (const auto& task : tasks) {
                const double* xpos  = &data->xpos[3 * task.bodyId];
                const double* xquat = &data->xquat[4 * task.bodyId];
                const Eigen::Vector3d currPos(xpos[0], xpos[1], xpos[2]);
                Eigen::Quaterniond currRot(xquat[0], xquat[1], xquat[2], xquat[3]);
                currRot.normalize();

                const Eigen::Vector3d posErr = task.targetPos - currPos;
                const Eigen::Vector3d rotErr = retarget_internal::computeOrientationErrorWorld(currRot, task.targetRot);
                sqErr += posErr.squaredNorm() + rotErr.squaredNorm();
            }

            return std::sqrt(sqErr);
        }

        void solveTaskSet(const std::vector<MujocoLegacyTaskRuntime>& tasks) {
            if (tasks.empty()) {
                return;
            }

            const int nv    = model->nv;
            const double dt = options.integrationTimestep;
            if (dt <= 1e-12) {
                throw std::runtime_error("integrationTimestep must be positive.");
            }
            const double invDt = 1.0 / dt;

            double currError = computeTaskError(tasks);
            solver::QPSolver solver;

            std::vector<mjtNum> jacp(3 * nv);
            std::vector<mjtNum> jacr(3 * nv);

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

                for (int j = 0; j < model->njnt; ++j) {
                    const int jointType = model->jnt_type[j];
                    if (jointType != mjJNT_HINGE && jointType != mjJNT_SLIDE) {
                        continue;
                    }
                    if (model->jnt_limited[j] <= 0) {
                        continue;
                    }

                    const int qadr    = model->jnt_qposadr[j];
                    const int vadr    = model->jnt_dofadr[j];
                    const double qmin = model->jnt_range[2 * j + 0];
                    const double qmax = model->jnt_range[2 * j + 1];

                    qp.ciLb[vadr] = std::max(qp.ciLb[vadr], (qmin - data->qpos[qadr]) / dt);
                    qp.ciUb[vadr] = std::min(qp.ciUb[vadr], (qmax - data->qpos[qadr]) / dt);
                }

                for (const auto& task : tasks) {
                    std::fill(jacp.begin(), jacp.end(), 0.0);
                    std::fill(jacr.begin(), jacr.end(), 0.0);
                    mj_jacBody(model.get(), data.get(), jacp.data(), jacr.data(), task.bodyId);

                    const Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> Jp(jacp.data(), 3, nv);
                    const Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> Jr(jacr.data(), 3, nv);

                    const double* xpos  = &data->xpos[3 * task.bodyId];
                    const double* xquat = &data->xquat[4 * task.bodyId];
                    const Eigen::Vector3d currPos(xpos[0], xpos[1], xpos[2]);
                    Eigen::Quaterniond currRot(xquat[0], xquat[1], xquat[2], xquat[3]);
                    currRot.normalize();

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

                qvel = out.x;

                mj_integratePos(model.get(), data->qpos, qvel.data(), dt);
                for (int j = 0; j < model->njnt; ++j) {
                    const int jointType = model->jnt_type[j];
                    if ((jointType == mjJNT_HINGE || jointType == mjJNT_SLIDE) && model->jnt_limited[j] > 0) {
                        const int qadr    = model->jnt_qposadr[j];
                        const double qmin = model->jnt_range[2 * j + 0];
                        const double qmax = model->jnt_range[2 * j + 1];
                        data->qpos[qadr]  = std::min(std::max(data->qpos[qadr], qmin), qmax);
                    }
                }
                mju_copy(data->qvel, qvel.data(), model->nv);
                mj_forward(model.get(), data.get());
                syncQposFromData();

                const double nextError = computeTaskError(tasks);
                if (currError - nextError <= options.progressThreshold) {
                    break;
                }
                currError = nextError;
            }
        }
    };

    MujocoLegacyRetargetBackend::MujocoLegacyRetargetBackend(const std::filesystem::path& robotModelPath, IkConfig ikConfig,
                                                             RetargetOptions options)
        : impl_(std::make_unique<Impl>(robotModelPath, std::move(ikConfig), std::move(options))) {}

    MujocoLegacyRetargetBackend::~MujocoLegacyRetargetBackend() = default;

    Eigen::VectorXd MujocoLegacyRetargetBackend::retargetFrame(const HumanFrame& humanFrame, bool offsetToGround) {
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

    HumanFrame MujocoLegacyRetargetBackend::prepareHumanFrame(const HumanFrame& humanFrame, bool offsetToGround) const {
        return impl_->prepareHumanFrame(humanFrame, offsetToGround);
    }

    void MujocoLegacyRetargetBackend::setQpos(const Eigen::VectorXd& qpos) {
        if (qpos.size() != impl_->model->nq) {
            throw std::runtime_error("setQpos size mismatch.");
        }

        mju_copy(impl_->data->qpos, qpos.data(), impl_->model->nq);
        mju_zero(impl_->data->qvel, impl_->model->nv);
        impl_->qvel.setZero();
        mj_forward(impl_->model.get(), impl_->data.get());
        impl_->syncQposFromData();
    }

    const Eigen::VectorXd& MujocoLegacyRetargetBackend::currentQpos() const { return impl_->qpos; }

    bool MujocoLegacyRetargetBackend::hasRootFreeFlyer() const { return impl_->hasRootFreeFlyer; }

    const std::vector<ScalarJointCoordinate>& MujocoLegacyRetargetBackend::scalarJointCoordinates() const {
        return impl_->scalarJointCoordinates;
    }

}  // namespace gmr
