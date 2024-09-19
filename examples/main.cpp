#include <time.h>
#include <Eigen/Core>
#include <iostream>
#include <liegroups/liegroups.hpp>

int main(int argc, char** argv) {
  srand(time(NULL));

  std::cout << "hello sophus lie" << std::endl;

  std::cout << LieGroups::greet("dev") << std::endl;

  Eigen::Vector3f x;
  x << 1, 2, 3;

  std::cout << "x : " << x.transpose() << std::endl;

  Eigen::Vector3f y = LieGroups::times2(x);
  std::cout << "y " << y.transpose() << std::endl;

  // if (0) {
  //   // create a transform
  //   auto T = LieGroups::Matrix4f();
  //   T.setIdentity();

  //   std::cout << "T: " << T.matrix() << std::endl;

  //   // create a Vector6
  //   auto tau = 1e-2 * LieGroups::Vector6f::Random();
  //   std::cout << "tau: " << tau.transpose() << std::endl;

  //   // return the exp
  //   T = LieGroups::SE3::Exp(tau);
  //   std::cout << "T: " << T.matrix() << std::endl;

  //   // convert it back using log
  //   auto tau2 = LieGroups::SE3::Log(T);
  //   std::cout << "tau2: " << tau2.transpose() << std::endl;
  // }

  // if (0) {
  //   using namespace LieGroups;
  //   Vector3f tau = 1e-2 * Vector3f::Random();
  //   Matrix3f R = SO3::Exp(tau);
  //   Vector3f v = SO3::Log(R);

  //   std::cout << "tau : " << tau.transpose() << std::endl;
  //   std::cout << "v   : " << v.transpose() << std::endl;
  // }

  if (1) {
    using namespace LieGroups;
    Vector6f tau = 1e-2 * Vector6f::Random();
    Matrix4f R = SE3::Exp(tau);
    Vector6f v = SE3::Log(R);

    std::cout << "tau : " << tau.transpose() << std::endl;
    std::cout << "v   : " << v.transpose() << std::endl;
  }
}
