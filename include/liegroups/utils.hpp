#pragma once
#include "types.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {

Matrix3f skew(Vector3f v) {
  Matrix3f T;
  T << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return T;
}

Vector3f deskew(Matrix3f T) {
  Vector3f v;
  v << T(2, 1), T(0, 2), T(1, 0);
  return v;
}

}  // namespace LieGroups
