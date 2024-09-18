#include <gtest/gtest.h>
#include <iostream>
#include <liegroups.hpp>

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
