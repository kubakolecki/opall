#pragma once

#include "opall/CostFunctionIdentifier.hpp"
#include "opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp"
#include "opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp"
#include "opall/cost_function_data/PointToPointCostFunctionData.hpp"
#include "opall/cost_function_data/PointToPointFixedStationCostFunctionData.hpp"

#include <string>

namespace opall::cost_function_id_generator
{

CostFunctionIdentifier generate(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData);
CostFunctionIdentifier generate(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData);

// TODO: implement when defined
// CostFunctionId generate(const cost_function_data::PointToPointCostFunctionData& costFunctionData);
// CostFunctionId generate(const cost_function_data::PointToPointFixedStationCostFunctionData& costFunctionData);

} // namespace opall::cost_function_id_generator
