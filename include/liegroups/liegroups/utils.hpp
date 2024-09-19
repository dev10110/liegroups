#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>
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

template <typename F, int N>
bool is_symmetric_posdef(Eigen::Matrix<F, N, N> A) {
  typedef Eigen::Matrix<F, N, N> Mat;

  Eigen::LLT<Mat> A_llt(A);
  if (!A.isApprox(A.transpose()) || A_llt.info() == Eigen::NumericalIssue) {
    return false;
  }
  return true;
}

// randoms

// @brief: creates a zero-mean random gaussian vector with identity covariance
template <typename F, int N>
Eigen::Vector<F, N> randn() {
  // create a standard normal distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dist(0.0, 1.0);

  // sample zero-mean normal vector
  Eigen::Vector<F, N> z;
  for (int i = 0; i < N; ++i) {
    z(i) = dist(gen);
  }

  return z;
}

template <typename F, int N>
Eigen::Vector<F, N> randn(const Eigen::Matrix<F, N, N>& Sigma) {
  typedef Eigen::Matrix<F, N, N> Mat;
  typedef Eigen::Vector<F, N> Vec;

  // check if the Sigma is essentially 0
  if (Sigma.norm() <= 1e-10) {
    return Vec::Zero();
  }

  // first check the sigma
  Eigen::LLT<Mat> Sigma_llt(Sigma);
  if (!Sigma.isApprox(Sigma.transpose()) ||
      Sigma_llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Possibly non semi-positive definitie matrix!");
  }

  // get unit random vector
  Vec z = randn<F, N>();

  // apply transform
  Vec x = Sigma_llt.matrixL() * z;

  return x;
}

template <typename F, int N>
Eigen::Vector<F, N> randn(const Eigen::Vector<F, N>& mu,
                          const Eigen::Matrix<F, N, N>& Sigma) {
  return mu + randn(Sigma);
}

}  // namespace LieGroups
