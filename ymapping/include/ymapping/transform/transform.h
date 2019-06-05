#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ymapping/transform/rigid_transform.h"

namespace transform {

// Returns the non-negative rotation angle in radians of the 3D transformation
// 'transform'.
template <typename FloatType>
FloatType GetAngle(const Rigid2<FloatType>& transform) {
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}

// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
//以给定3D'旋转'的弧度返回偏航分量。 假设“旋转”由绕X，Y然后Z的三次旋转组成，则返回绕Z旋转的角度。
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}


}  // namespace transform

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_H_
