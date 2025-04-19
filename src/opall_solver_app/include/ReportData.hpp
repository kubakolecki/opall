#pragma once

#include <opall/OptimizationDataContainer.hpp> //TODO: unify <>, "" for include in opall solver app project
#include <opall/SolverSummary.hpp>
#include <opall/residual_computation.hpp>

#include <optional>

namespace opall_solver_app
{

class ReportData
{
  public:
    ReportData(const opall::optimization_data_container::OptimizationDataContainer &optimizationDataContainer,
               const opall::SolverSummary &solverSummary,
               std::optional<std::reference_wrapper<const opall::ResidualContainer>> residuals);
    const opall::optimization_data_container::OptimizationDataContainer &getOptimizationDataContainer() const;
    const opall::SolverSummary &getSolverSummary() const;
    const std::optional<std::reference_wrapper<const opall::ResidualContainer>> &getResiduals() const;

  private:
    const opall::optimization_data_container::OptimizationDataContainer &m_optimizationDataContainer;
    const opall::SolverSummary &m_solverSummary;
    const std::optional<std::reference_wrapper<const opall::ResidualContainer>> m_residuals{std::nullopt};
};

} // namespace opall_solver_app