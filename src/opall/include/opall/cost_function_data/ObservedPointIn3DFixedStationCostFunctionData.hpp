#pragma once

#include <Eigen/Core>

#include "opall/LossFunctionDescription.hpp"
#include "opall/data_types.hpp"

namespace opall::cost_function_data
{
struct ObservedPointIn3DFixedStationCostFunctionData
{
    PoseId poseId;
    PointId pointId;
    point_3d::Point observedPoint;
    Pose pose;
    uncertainty3d::SquareRootInformationMatrix uncertainty;
    double *point;
    LossFunctionDescription lossFunctionDescription;
};
} // namespace opall::cost_function_data