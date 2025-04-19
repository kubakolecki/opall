#include "opall/geometry.hpp"

#include <Eigen/Core>

namespace opall
{
QuaternionWxyz quaternionConjungate(const QuaternionWxyz &quaternion) noexcept
{
    QuaternionWxyz quaternionConjungate{quaternion};
    quaternionConjungate(1) = -quaternion(1);
    quaternionConjungate(2) = -quaternion(2);
    quaternionConjungate(3) = -quaternion(3);
    return quaternionConjungate;
}

point_3d::Point rotatePoint(const QuaternionWxyz &quaternionWxyz, const point_3d::Point &point) noexcept
{
    auto rotatedPoint{point_3d::Point{}};
    double uv0 = quaternionWxyz(2) * point(2) - quaternionWxyz(3) * point(1);
    double uv1 = quaternionWxyz(3) * point(0) - quaternionWxyz(1) * point(2);
    double uv2 = quaternionWxyz(1) * point(1) - quaternionWxyz(2) * point(0);
    uv0 += uv0;
    uv1 += uv1;
    uv2 += uv2;
    rotatedPoint(0) = point(0) + quaternionWxyz(0) * uv0;
    rotatedPoint(1) = point(1) + quaternionWxyz(0) * uv1;
    rotatedPoint(2) = point(2) + quaternionWxyz(0) * uv2;
    rotatedPoint(0) += quaternionWxyz(2) * uv2 - quaternionWxyz(3) * uv1;
    rotatedPoint(1) += quaternionWxyz(3) * uv0 - quaternionWxyz(1) * uv2;
    rotatedPoint(2) += quaternionWxyz(1) * uv1 - quaternionWxyz(2) * uv0;
    return rotatedPoint;
}

point_3d::Point transformPointUsingPose(const point_3d::Point &point, const Pose &pose) noexcept
{
    const auto transformedPoint{rotatePoint(pose.quaternionWxyz, point) + pose.position};
    return transformedPoint;
}

} // namespace opall