#pragma once

#include <Eigen/Dense>

namespace gmr {

using Scalar = double;
using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Matrix = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
using Matrix3x = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;

}  // namespace gmr
