#pragma once

#include "types.hpp"
#include "utils.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {
namespace SO3 {

Matrix3f Identity() { return Matrix3f::Identity(); }

Matrix3f Exp(Vector3f tau) {
  float theta = tau.norm();

  if (theta == 0)
  {
	  return Matrix3f::Identity();
  }

  Vector3f u = tau / theta;

  Matrix3f S = skew(u);

  Matrix3f R = Matrix3f::Identity() + std::sin(theta) * S +
               (1 - std::cos(theta)) * S * S;

  return R;
}

Vector3f Log(Matrix3f R) {
  float theta = std::acos((R.trace() - 1) / 2);

  if (theta == 0){
	  return Vector3f::Zero();
  }

  return (theta / (2 * std::sin(theta))) * deskew(R - R.transpose());
}

Vector3f action(Matrix3f R, Vector3f v) { return R * v; }

}  // namespace SO3

}  // namespace LieGroups
