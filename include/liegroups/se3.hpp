#pragma once

#include "types.hpp"
#include "utils.hpp"
#include "so3.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {
namespace SE3 {

Matrix4f Identity() { return Matrix4f::Identity(); }

Matrix3f V(Vector3f thetau) {
  float theta = thetau.norm();
  if (theta == 0)
  {
	  return Matrix3f::Identity();
  }

  float a = (1 - std::cos(theta)) / (std::pow(theta, 2));
  float b = (theta - std::sin(theta)) / (std::pow(theta, 3));

  Vector3f u = thetau / theta;
  Matrix3f S = skew(thetau);

  return Matrix3f::Identity() + a * S + b * S * S;
}

Matrix4f Exp(Vector6f tau) {

  // extract
  Vector3f rho = tau.head(3);
  Vector3f thetau = tau.tail(3);

  // initialize
  Matrix4f T = Matrix4f();
  T.setIdentity();

  // update the blocks
  T.block<3, 3>(0, 0) = SO3::Exp(thetau);
  T.block<3, 1>(0, 3) = SE3::V(thetau) * rho;

  return T;
}

Vector6f Log(Matrix4f T) {
  Matrix3f R = T.block<3, 3>(0, 0);
  Vector3f t = T.block<3, 1>(0, 3);

  Vector3f thetau = SO3::Log(R);

  Matrix3f Vtheta = SE3::V(thetau);

  Vector3f rho = Vtheta.inverse() * t;

  Vector6f tau;
  tau.head(3) = rho;
  tau.tail(3) = thetau;

  return tau;
}

Vector3f action(Matrix4f T, Vector3f v) {
  // make homogeneous
  Vector4f vh;
  vh.head(3) = v;
  vh(3) = 1;

  // action
  Vector4f qh = T * vh;
  Vector3f q = qh.head(3);

  return q;
}

Vector4f action(Matrix4f T, Vector4f v) { return T * v; }

}  // namespace SE3

}  // namespace LieGroups
