#pragma once

#include "gmr/solver/qp_data.h"
#include "gmr/solver/qp_solver.h"

namespace gmr::solver {

class HQPSolver {
 public:
  HQPSolver() = default;
  ~HQPSolver() = default;

  const QPOutput& solve(HQPData& hqpData);

 private:
  QPSolver qp0Solver_;
  QPSolver qp1Solver_;

  QPOutput output_;
};

}  // namespace gmr::solver
