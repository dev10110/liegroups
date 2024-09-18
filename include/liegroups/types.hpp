#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {

///////////
// TYPES //
///////////
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;

}  // namespace LieGroups
