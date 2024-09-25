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
Matrix<F, 3, 3> rotation(Matrix<F, 4, 4> T) {
  Matrix<F, 3, 3> R = T.block(0, 0, 3, 3);
  return R;
}

template <typename F>
Vector<F, 3> translation(Matrix<F, 4, 4> T) {
  Vector<F, 3> t = T.block(0, 3, 3, 1);
  return t;
}

template <typename F>
Matrix<F, 3, 3> V(Vector<F, 3> thetau) {
  F theta = thetau.norm();
  if (theta == 0) {
    return Matrix3f::Identity();
  }

  F a = (1 - std::cos(theta)) / (std::pow(theta, 2));
  F b = (theta - std::sin(theta)) / (std::pow(theta, 3));

  Matrix<F, 3, 3> S = skew(thetau);

  return Matrix<F, 3, 3>::Identity() + a * S + b * S * S;
}

template <typename F>
Matrix<F, 4, 4> hat(Vector<F, 6> v) {
  Vector<F, 3> rho = v.head(3);
  Vector<F, 3> thetau = v.tail(3);

  Matrix<F, 4, 4> tau = Matrix<F, 4, 4>::Zero();
  tau.block(0, 0, 3, 3) = skew(thetau);
  tau.block(0, 3, 3, 1) = rho;

  return tau;
}

template <typename F>
Vector<F, 6> vee(Matrix<F, 4, 4> tau) {
  Vector<F, 3> rho = tau.block(0, 3, 3, 1);
  Vector<F, 3> thetau = deskew(Matrix<F, 3, 3>(tau.block(0, 0, 3, 3)));

  Vector<F, 6> v;
  v.head(3) = rho;
  v.tail(3) = thetau;

  return v;
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
Matrix<F, 6, 6> adjoint_matrix(Matrix<F, 4, 4> T) {
  Matrix<F, 3, 3> R = rotation(T);
  Vector<F, 3> t = translation(T);

  Matrix<F, 6, 6> A = Matrix<F, 6, 6>::Zeros();

  A.block(0, 0, 3, 3) = R;
  A.block(0, 3, 3, 3) = skew(t) * R;
  A.block(3, 3, 3, 3) = R;

  return A;
}

template <typename F>
Vector<F, 3> action(Matrix<F, 4, 4> T, Vector<F, 3> v) {
  Matrix<F, 3, 3> R = rotation(T);
  Vector<F, 3> t = translation(T);

  return R * v + t;
}

template <typename F>
Vector<F, 4> action(Matrix<F, 4, 4> T, Vector<F, 4> v) {
  return T * v;
}

template <typename F>
Matrix<F, 6, 6> action(Matrix<F, 4, 4> T, Vector<F, 3> v, Matrix<F, 3, 6>& JT,
                       Matrix<F, 3, 3>& Jv) {
  Matrix<F, 3, 3> R = rotation(T);

  JT.block(0, 0, 3, 3) = R;
  JT.block(0, 3, 3, 3) = -R * skew(v);

  Jv = R;

  return action(T, v);
}

template <typename F>
Matrix<F, 4, 4> inverse(Matrix<F, 4, 4> T) {
  Matrix<F, 3, 3> R = rotation(T);
  Vector<F, 3> t = translation(T);

  Matrix<F, 4, 4> invT = Matrix<F, 4, 4>::Identity();
  invT.block(0, 0, 3, 3) = R.transpose();
  invT.block(0, 3, 3, 1) = -R.transpose() * t;

  return invT;
}

template <typename F>
Matrix<F, 4, 4> inverse(Matrix<F, 4, 4> T, Matrix<F, 6, 6>& jacobian) {
  jacobian = -adjoint_matrix(T);
  return inverse(T);
}

template <typename F>
Matrix<F, 4, 4> compose(Matrix<F, 4, 4> Ta, Matrix<F, 4, 4> Tb) {
  Matrix<F, 3, 3> Ra = rotation(Ta);
  Matrix<F, 3, 3> Rb = rotation(Tb);
  Vector<F, 3> ta = translation(Ta);
  Vector<F, 3> tb = translation(Tb);

  Matrix<F, 4, 4> T = Matrix<F, 4, 4>::Identity();
  T.block(0, 0, 3, 3) = Ra * Rb;
  T.block(0, 3, 3, 1) = ta + Ra * tb;

  return T;
}

template <typename F>
Matrix<F, 4, 4> compose(Matrix<F, 4, 4> Ta, Matrix<F, 4, 4> Tb,
                        Matrix<F, 6, 6>& Ja, Matrix<F, 6, 6>& Jb) {
  Matrix<F, 3, 3> Ra = rotation(Ta);
  Matrix<F, 3, 3> Rb = rotation(Tb);
  Vector<F, 3> ta = translation(Ta);
  Vector<F, 3> tb = translation(Tb);

  Matrix<F, 3, 3> Rbt = Rb.tranpose();

  Ja.block(0, 0, 3, 3) = Rbt;
  Ja.block(0, 3, 3, 3) = -Rbt * skew(tb);
  Ja.block(3, 0, 3, 3).setZero();
  Ja.block(3, 3, 3, 3) = Rbt;

  Jb = Matrix<F, 6, 6>::Identity();

  return compose(Ta, Tb);
}

}  // namespace SE3

}  // namespace LieGroups
