#pragma once

#include <vector>

#include "opall/cost_function_data/CostFunctionData.hpp"

namespace opall::cost_function_data
{

class CostFunctionDataContainer
{
  private:
    std::vector<CostFunctionData> data;

  public:
    CostFunctionDataContainer();
    CostFunctionDataContainer(size_t capacity);
    void insert(const CostFunctionData &costFunctionData);
    void insert(CostFunctionData &&costFunctionData);
    const std::vector<CostFunctionData> &getData() const;
};

} // namespace opall::cost_function_data