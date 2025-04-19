#pragma once

#include <memory>

#include "opall/OptimizationConfig.hpp"
#include "opall/SolverSummary.hpp"
#include "opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp"
#include "opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp"

namespace opall
{

class Problem
{
  public:
    Problem();
    ~Problem();

    Problem(Problem const &other) = delete;
    Problem &operator=(Problem const &other) = delete;

    Problem(Problem &&other) noexcept;
    Problem &operator=(Problem &&other) noexcept;

    void insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData);
    void insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData);

    opall::SolverSummary solve(const OptimizationConfig &config = OptimizationConfig{});

  private:
    struct Impl;
    std::unique_ptr<Impl> const pimpl;
};

} // namespace opall
