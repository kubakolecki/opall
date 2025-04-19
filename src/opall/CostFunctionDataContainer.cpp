#include "opall/CostFunctionDataContainer.hpp"

namespace opall::cost_function_data
{
CostFunctionDataContainer::CostFunctionDataContainer()
{
    data.reserve(4096);
}

CostFunctionDataContainer::CostFunctionDataContainer(size_t capacity)
{
    data.reserve(capacity);
}

void CostFunctionDataContainer::insert(const CostFunctionData &costFunctionData)
{
    data.push_back(costFunctionData);
}

void CostFunctionDataContainer::insert(CostFunctionData &&costFunctionData)
{
    data.emplace_back(std::move(costFunctionData));
}

const std::vector<CostFunctionData> &CostFunctionDataContainer::getData() const
{
    return data;
}
} // namespace opall::cost_function_data