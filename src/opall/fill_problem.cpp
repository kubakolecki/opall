#include "opall/fill_problem.hpp"
#include "opall/cost_function_data/CostFunctionData.hpp"

namespace opall
{

void fillProblem(const cost_function_data::CostFunctionDataContainer &costFunctionDataContainer, Problem &problem)
{
    for (const auto &costFunctionData : costFunctionDataContainer.getData())
    {
        cost_function_data::insertCostFunctionToProblem(costFunctionData, problem);
    }
}

} // namespace opall
