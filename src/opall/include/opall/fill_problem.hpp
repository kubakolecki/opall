#pragma once

#include "opall/CostFunctionDataContainer.hpp"
#include "opall/Problem.hpp"

namespace opall
{
void fillProblem(const cost_function_data::CostFunctionDataContainer &costFunctionDataContainer, Problem &problem);
}