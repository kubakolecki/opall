#pragma once

#include "data_types.hpp"

namespace opall
{
QuaternionWxyz quaternionConjungate(const QuaternionWxyz &quaternion) noexcept;
point_3d::Point rotatePoint(const QuaternionWxyz &quaternion, const point_3d::Point &point) noexcept;
point_3d::Point transformPointUsingPose(const point_3d::Point &point, const Pose &pose) noexcept;

} // namespace opall