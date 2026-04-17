#pragma once

// Core QP/HQP data layout adapted from:
// ~/Workspace/SingoriX/Experimental/whole_body_control

#include <Eigen/Dense>
#include <sstream>

#include "gmr/retarget/types.h"

namespace gmr::solver {

struct QPData {
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H;
  Vector g;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> CI;
  Vector ciLb;
  Vector ciUb;

  void reset(int nVar, int nIneq) {
    H.setZero(nVar, nVar);
    g.setZero(nVar);
    CI.setZero(nIneq, nVar);
    ciLb.setZero(nIneq);
    ciUb.setZero(nIneq);
  }

  std::string toString() const {
    std::ostringstream oss;
    oss << "H:\n" << H << "\n";
    oss << "g:\n" << g.transpose() << "\n";
    oss << "CI:\n" << CI << "\n";
    oss << "ciLb:\n" << ciLb.transpose() << "\n";
    oss << "ciUb:\n" << ciUb.transpose() << "\n";
    return oss.str();
  }
};

struct HQPData {
  QPData qp0;
  QPData qp1;
  Matrix J0;

  void reset(int nVar, int nEq0, int nIneq0, int nIneq1 = 0) {
    qp0.reset(nVar, nIneq0);
    qp1.reset(nVar, nIneq1 + nIneq0 + nEq0);
    J0.setZero(nEq0, nVar);
  }
};

enum class QPStatus {
  kUnknown = -1,
  kOptimal = 0,
  kInfeasible = 1,
  kUnbounded = 2,
  kMaxIterReached = 3,
  kError = 4,
};

struct QPOutput {
  QPStatus status = QPStatus::kUnknown;
  Vector x;
  Vector lambda;
  Eigen::VectorXi activeSet;
  int iterations = 0;

  void resize(int nVar, int nEq, int nIneq) {
    x.resize(nVar);
    lambda.resize(nEq + nIneq);
    activeSet.resize(nIneq);
  }
};

}  // namespace gmr::solver
