#include "gmr/solver/hqp_solver.h"

// HQP layering adapted from:
// ~/Workspace/SingoriX/Experimental/whole_body_control/src/solver/hqp_solver.cpp

namespace gmr::solver {

const QPOutput& HQPSolver::solve(HQPData& hqpData) {
  auto& qp0 = hqpData.qp0;
  auto& J0 = hqpData.J0;
  auto& qp1 = hqpData.qp1;

  const int nVar = qp0.H.rows();
  const int nEq0 = J0.rows();
  const int nIn0 = qp0.CI.rows();
  const int nIn1 = qp1.CI.rows() - nIn0 - nEq0;

  const QPOutput& sol0 = qp0Solver_.solve(qp0);
  if (sol0.status != QPStatus::kOptimal) {
    output_.status = sol0.status;
    output_.x = Vector::Zero(nVar);
    output_.iterations = sol0.iterations;
    return output_;
  }

  if (qp0.H.determinant() > 1e-8) {
    output_.status = QPStatus::kOptimal;
    output_.x = sol0.x;
    output_.iterations = sol0.iterations;
    return output_;
  }

  qp1.CI.middleRows(nIn1, nIn0) = qp0.CI;
  qp1.ciLb.middleRows(nIn1, nIn0) = qp0.ciLb;
  qp1.ciUb.middleRows(nIn1, nIn0) = qp0.ciUb;
  qp1.CI.bottomRows(nEq0) = J0;
  qp1.ciLb.tail(nEq0) = J0 * sol0.x;
  qp1.ciUb.tail(nEq0) = J0 * sol0.x;

  const QPOutput& sol1 = qp1Solver_.solve(qp1);
  output_.status = sol1.status;
  output_.x = sol1.x;
  output_.lambda = sol1.lambda;
  output_.activeSet = sol1.activeSet;
  output_.iterations = sol0.iterations + sol1.iterations;
  return output_;
}

}  // namespace gmr::solver
