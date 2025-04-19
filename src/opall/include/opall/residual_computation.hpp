#pragma once

#include "CostFunctionDataContainer.hpp"
#include "evaluators.hpp"

#include <vector>

namespace opall
{

using ResidualContainer = std::vector<std::pair<CostFunctionIdentifier, EvaluationResult>>;
ResidualContainer computeResiduals(const cost_function_data::CostFunctionDataContainer &costFunctionData);

} // namespace opall
