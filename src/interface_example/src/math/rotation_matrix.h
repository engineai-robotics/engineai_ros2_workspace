#pragma once

#include <Eigen/Dense>
namespace math {

Eigen::Vector3d CalcRollPitchYawFromRotationMatrix(const Eigen::Matrix3d& R);
}  // namespace math