#include <gtest/gtest.h>
#include <iostream>
#include <liegroups/liegroups.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace LieGroups;

// basic assertion
TEST(LieGroupsTest, BasicAssertions) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(7 * 6, 42);
}

// Exp(zero) == Identity
TEST(LieGroupsTest, ExpZeroSO3) {
  Vector3f v = Vector3f::Zero();
  Matrix3f R = SO3::Exp(v);
  EXPECT_EQ(R, Matrix3f::Identity());
}

TEST(LieGroupsTest, ExpZeroSE3) {
  Vector6f v = Vector6f::Zero();
  Matrix4f T = SE3::Exp(v);
  EXPECT_EQ(T, Matrix4f::Identity());
}

// Log(Identity) == Zero
TEST(LieGroupsTest, LogIdentity) {
  Matrix3f R = SO3::Identity<float>();
  Vector3f v = SO3::Log(R);
  EXPECT_EQ(v, Vector3f::Zero());
}

TEST(LieGroupsTest, LogIdentitySE3) {
  Matrix4f T = SE3::Identity<float>();
  Vector6f v = SE3::Log(T);
  EXPECT_EQ(v, Vector6f::Zero());
}

// Exp and Log are inverses
TEST(LieGroupsTest, ExpLogSO3) {
  // use a fixed seed;
  srand(0);

  for (int i = 0; i < 100; i++) {
    Vector3f tau = Vector3f::Random();
    Matrix3f R = SO3::Exp(tau);
    EXPECT_NEAR((SO3::Log(R) - tau).norm(), 0, 1e-6);
  }
}

// Exp and Log are inverses
TEST(LieGroupsTest, ExpLogSE3) {
  // use a fixed seed;
  srand(0);

  for (int i = 0; i < 100; i++) {
    Vector6f tau = Vector6f::Random();
    Matrix4f T = SE3::Exp(tau);
    EXPECT_NEAR((SE3::Log(T) - tau).norm(), 0, 1e-6);
  }
}

TEST(LieGroupsTest, HatVeeSO3) {
  srand(0);
  for (int i = 0; i < 100; ++i) {
    Vector3f v = Vector3f::Random();
    Matrix3f tau = SO3::hat(v);
    Vector3f v2 = SO3::vee(tau);
    EXPECT_NEAR((v2 - v).norm(), 0, 1e-6);
  }
}

TEST(LieGroupsTest, HatVeeSE3) {
  srand(0);
  for (int i = 0; i < 100; ++i) {
    Vector6f v = Vector6f::Random();
    Matrix4f tau = SE3::hat(v);
    Vector6f v2 = SE3::vee(tau);
    EXPECT_NEAR((v2 - v).norm(), 0, 1e-6);
  }
}

TEST(LieGroupsTest, ExpIsMatrixExpSO3) {
  // use a fixed seed
  srand(0);

  for (int i = 0; i < 100; i++) {
    Vector3f v = Vector3f::Random();
    Matrix3f tau = SO3::hat(v);  // get the element of the lie algebra
    Matrix3f T = SO3::Exp(v);

    Matrix3f T2 = tau.exp();

    EXPECT_NEAR((T2 - T).norm(), 0, 1e-6);
  }
}

TEST(LieGroupsTest, ExpIsMatrixExpSE3) {
  // use a fixed seed
  srand(0);

  for (int i = 0; i < 100; i++) {
    Vector6f v = Vector6f::Random();
    Matrix4f tau = SE3::hat(v);  // get the element of the lie algebra
    Matrix4f T = SE3::Exp(v);

    Matrix4f T2 = tau.exp();

    EXPECT_NEAR((T2 - T).norm(), 0, 1e-6);
  }
}
