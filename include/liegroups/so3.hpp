#pragma once

#include "types.hpp"
#include "utils.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {
namespace SO3 {

using namespace Eigen;

template <typename F>
Matrix<F, 3, 3> Identity() {
  return Matrix<F, 3, 3>::Identity();
}

template <typename F>
Matrix<F, 3, 3> Exp(Eigen::Vector<F, 3> tau) {
  typedef Matrix<F, 3, 3> Matrix3;

  F theta = tau.norm();

  if (theta == 0) {
    return Matrix3::Identity();
  }

  Vector3f u = tau / theta;

  Matrix3f S = skew(u);

  Matrix3f R =
      Matrix3::Identity() + std::sin(theta) * S + (1 - std::cos(theta)) * S * S;

  return R;
}

template <typename F>
Vector<F, 3> Log(Matrix<F, 3, 3> R) {
  F theta = std::acos((R.trace() - 1) / 2);

  if (theta == 0) {
    return Vector<F, 3>::Zero();
  }

  Matrix<F, 3, 3> dR = R - R.transpose();

  return (theta / (2 * std::sin(theta))) * deskew(dR);
}

template <typename F>
Vector<F, 3> action(Matrix<F, 3, 3> R, Vector<F, 3> v) {
  return R * v;
}

}  // namespace SO3

}  // namespace LieGroups
