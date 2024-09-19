#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "types.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {

using namespace Eigen;

template <typename F>
Matrix<F, 3, 3> skew(Vector<F, 3> v) {
  Matrix<F, 3, 3> T;
  T << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return T;
}

template <typename F>
Vector<F, 3> deskew(Matrix<F, 3, 3> T) {
  Vector<F, 3> v;
  v << T(2, 1), T(0, 2), T(1, 0);
  return v;
}

}  // namespace LieGroups
