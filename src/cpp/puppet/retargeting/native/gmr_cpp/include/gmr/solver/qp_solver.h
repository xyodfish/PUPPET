#pragma once

#include <memory>

#include <qpOASES.hpp>

#include "gmr/solver/qp_data.h"

namespace gmr::solver {

class QPSolver {
 public:
  QPSolver();
  ~QPSolver() = default;

  const QPOutput& solve(const QPData& qpData);

 private:
  std::shared_ptr<qpOASES::SQProblem> solver_;
  qpOASES::Options options_;

  bool initialized_ = false;
  int nv_ = 0;
  int nc_ = 0;

  QPOutput output_;
};

}  // namespace gmr::solver
