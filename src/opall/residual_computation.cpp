#include "opall/residual_computation.hpp"
#include "opall/cost_function_data/CostFunctionData.hpp"
#include "opall/cost_function_data_id_generator.hpp"

#include <algorithm>

namespace opall
{
ResidualContainer opall::computeResiduals(const cost_function_data::CostFunctionDataContainer &costFunctionData)
{
    ResidualContainer residuals;
    residuals.reserve(costFunctionData.getData().size());
    std::ranges::transform(costFunctionData.getData(), std::back_inserter(residuals), [](const auto &data) {
        return std::make_pair(cost_function_data::generateIdentifier(data), cost_function_data::evaluate(data));
    });
    return residuals;
}

} // namespace opall
