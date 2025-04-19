#pragma once

#include "CostFunctionDataContainer.hpp"
#include "ModellingConfig.hpp"
#include "OptimizationDataContainer.hpp"
#include "cost_function_data/ObservedPointIn3DCostFunctionData.hpp"
#include "cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp"
#include "evaluators.hpp"

namespace opall::cost_function_data
{
struct ObservedPointIn3DStrategy
{
};

struct PointToPointStrategy
{
};

CostFunctionDataContainer createCostFunctionData(optimization_data_container::OptimizationDataContainer &optimizationDataContainer,
                                                 const ModellingConfig &config, [[maybe_unused]] ObservedPointIn3DStrategy &&strategy);

} // namespace opall::cost_function_data