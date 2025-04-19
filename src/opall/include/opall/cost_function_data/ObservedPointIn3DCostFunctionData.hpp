#pragma once

#include <Eigen/Core>

#include "opall/LossFunctionDescription.hpp"
#include "opall/data_types.hpp"

namespace opall::cost_function_data
{

struct ObservedPointIn3DCostFunctionData
{
    PoseId poseId;
    PointId pointId;
    point_3d::Point observedPoint;
    uncertainty3d::SquareRootInformationMatrix uncertainty;
    double *point;
    double *posePosition;
    double *poseWxyzQuaternion;
    LossFunctionDescription lossFunctionDescription;
};
} // namespace opall::cost_function_data