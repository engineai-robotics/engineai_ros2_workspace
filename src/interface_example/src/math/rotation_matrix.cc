#include <Eigen/Dense>
#include <cmath>
namespace math {

Eigen::Vector3d CalcRollPitchYawFromRotationMatrix(const Eigen::Matrix3d& R) {
  const Eigen::Quaterniond quaternion(R);
  using std::abs;
  using std::atan2;
  using std::sqrt;

  const double R22 = R(2, 2);
  const double R21 = R(2, 1);
  const double R10 = R(1, 0);
  const double R00 = R(0, 0);
  const double Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const double R20 = R(2, 0);
  const double q2 = atan2(-R20, Rsum);

  // Calculate q1 and q3 from Steps 2-6 (documented above).
  const double e0 = quaternion.w(), e1 = quaternion.x();
  const double e2 = quaternion.y(), e3 = quaternion.z();
  const double yA = e1 + e3, xA = e0 - e2;
  const double yB = e3 - e1, xB = e0 + e2;
  const double epsilon = Eigen::NumTraits<double>::epsilon();
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const double zA = (isSingularA ? double{0.0} : atan2(yA, xA));
  const double zB = (isSingularB ? double{0.0} : atan2(yB, xB));
  double q1 = zA - zB;  // First angle in rotation sequence.
  double q3 = zA + zB;  // Third angle in rotation sequence.

  q1 = (q1 > M_PI ? q1 - 2 * M_PI : q1);
  q1 = (q1 < -M_PI ? q1 + 2 * M_PI : q1);
  q3 = (q3 > M_PI ? q3 - 2 * M_PI : q3);
  q3 = (q3 < -M_PI ? q3 + 2 * M_PI : q3);

  // Return in (roll-pitch-yaw) order
  return Eigen::Vector3<double>(q1, q2, q3);
}

}  // namespace math