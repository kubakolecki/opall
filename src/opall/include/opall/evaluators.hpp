#pragma once

#include <Eigen/Core>

#include "cost_function_data/ObservedPointIn3DCostFunctionData.hpp"
#include "cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp"

namespace opall
{
struct EvaluationResult
{
    Eigen::VectorXd values;
};

struct ObservedPointIn3DFixedStationCostFunctionDataEvaluator
{
    EvaluationResult operator()(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData) const;
};

struct ObservedPointIn3DCostFunctionDataEvaluator
{
    EvaluationResult operator()(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData) const;
};
} // namespace opall