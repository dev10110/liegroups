#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "so3.hpp"
#include "types.hpp"
#include "utils.hpp"

// follows the notation in https://arxiv.org/pdf/1812.01537

namespace LieGroups {
namespace SE3 {
using namespace Eigen;

template <typename F>
Matrix<F, 4, 4> Identity() {
  return Matrix<F, 4, 4>::Identity();
}

template <typename F>
Matrix<F, 3, 3> V(Vector<F, 3> thetau) {
  F theta = thetau.norm();
  if (theta == 0) {
    return Matrix3f::Identity();
  }

  F a = (1 - std::cos(theta)) / (std::pow(theta, 2));
  F b = (theta - std::sin(theta)) / (std::pow(theta, 3));

  Vector<F, 3> u = thetau / theta;
  Matrix<F, 3, 3> S = skew(thetau);

  return Matrix<F, 3, 3>::Identity() + a * S + b * S * S;
}

template <typename F>
Matrix<F, 4, 4> Exp(Vector<F, 6> tau) {
  // extract
  Vector<F, 3> rho = tau.head(3);
  Vector<F, 3> thetau = tau.tail(3);

  // initialize
  Matrix<F, 4, 4> T = Matrix<F, 4, 4>::Identity();

  // update the blocks
  T.block(0, 0, 3, 3) = SO3::Exp(thetau);
  T.block(0, 3, 3, 1) = SE3::V(thetau) * rho;

  return T;
}

template <typename F>
Vector<F, 6> Log(Matrix<F, 4, 4> T) {
  Matrix<F, 3, 3> R = T.block(0, 0, 3, 3);
  Vector<F, 3> t = T.block(0, 3, 3, 1);

  Vector<F, 3> thetau = SO3::Log(R);

  Matrix<F, 3, 3> Vtheta = SE3::V(thetau);

  Vector<F, 3> rho = Vtheta.inverse() * t;

  Vector<F, 6> tau;
  tau.head(3) = rho;
  tau.tail(3) = thetau;

  return tau;
}

template <typename F>
Vector<F, 3> action(Matrix<F, 4, 4> T, Vector<F, 3> v) {
  // make homogeneous
  Vector<F, 4> vh;
  vh.head(3) = v;
  vh(3) = 1;

  // action
  Vector<F, 4> qh = T * vh;

  return qh.head(3);
}

template <typename F>
Vector<F, 4> action(Matrix<F, 4, 4> T, Vector<F, 4> v) {
  return T * v;
}

}  // namespace SE3

}  // namespace LieGroups
