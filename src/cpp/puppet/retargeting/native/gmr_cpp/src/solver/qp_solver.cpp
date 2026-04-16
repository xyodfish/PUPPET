#include "gmr/solver/qp_solver.h"

// QP solver implementation adapted from:
// ~/Workspace/SingoriX/Experimental/whole_body_control/src/solver/qp_solver.cpp

namespace gmr::solver {

QPSolver::QPSolver() {
  options_.setToMPC();
  options_.printLevel = qpOASES::PL_NONE;
  options_.enableEqualities = qpOASES::BT_TRUE;
  options_.enableRegularisation = qpOASES::BT_TRUE;
  options_.epsRegularisation = 1e-8;
}

const QPOutput& QPSolver::solve(const QPData& qpData) {
  const int nv = qpData.H.rows();
  const int nc = qpData.CI.rows();
  output_.resize(nv, 0, nc);

  if (!initialized_ || nv_ != nv || nc_ != nc) {
    solver_ = std::make_shared<qpOASES::SQProblem>(nv, nc);
    solver_->setOptions(options_);
    initialized_ = true;
    nv_ = nv;
    nc_ = nc;
  }

  int nWSR = 100;
  const qpOASES::returnValue ret = solver_->init(qpData.H.data(), qpData.g.data(), qpData.CI.data(), nullptr,
                                                  nullptr, qpData.ciLb.data(), qpData.ciUb.data(), nWSR);

  if (ret == qpOASES::SUCCESSFUL_RETURN) {
    output_.status = QPStatus::kOptimal;
    solver_->getPrimalSolution(output_.x.data());
    output_.iterations = 100 - nWSR;
  } else {
    output_.status = QPStatus::kError;
    if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
      output_.status = QPStatus::kMaxIterReached;
    }
    if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
      output_.status = QPStatus::kInfeasible;
    }
  }

  return output_;
}

}  // namespace gmr::solver
