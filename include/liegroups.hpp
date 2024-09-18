#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

// follows the notation in https://arxiv.org/pdf/1812.01537
#include "liegroups/se3.hpp"
#include "liegroups/so3.hpp"
#include "liegroups/types.hpp"
#include "liegroups/utils.hpp"

namespace LieGroups {

///////////
// TYPES //
///////////
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector4f Vector4f;
typedef Eigen::Vector<float, 6> Vector6f;

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Matrix4f Matrix4f;

///////////////
// UTILITIES //
///////////////
Matrix3f skew(Vector3f u);
Vector3f deskew(Matrix3f T);

/////////
// SO3 //
/////////
namespace SO3 {
Matrix3f Exp(Vector3f tau);
Vector3f Log(Matrix3f T);
Vector3f action(Matrix3f T, Vector3f v);
}  // namespace SO3

/////////
// SE3 //
/////////
namespace SE3 {
Matrix4f Identity();
Matrix3f V(Vector3f thetau);
Matrix4f Exp(Vector6f tau);
Vector6f Log(Matrix4f T);
Vector3f action(Matrix4f T, Vector3f v);
Vector4f action(Matrix4f T, Vector4f v);
}  // namespace SE3

}  // namespace LieGroups
