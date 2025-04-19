#pragma once

#include "ReportConfig.hpp"

#include <opall/ModellingConfig.hpp>
#include <opall/OptimizationConfig.hpp>

namespace opall_solver_app
{

struct Config
{
    enum class PointIn3DCorrespondeceType
    {
        POINT_TO_POINT,
        OBSERVED_POINT
    };

    PointIn3DCorrespondeceType pointIn3DCorrespondenceType;
    opall::ModellingConfig modellingConfig;
    opall::OptimizationConfig optimizationConfig;
    ReportConfig reportConfig;
};

} // namespace opall_solver_app
