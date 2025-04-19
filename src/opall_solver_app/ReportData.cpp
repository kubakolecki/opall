#include "ReportData.hpp"

opall_solver_app::ReportData::ReportData(const opall::optimization_data_container::OptimizationDataContainer &optimizationDataContainer,
                                         const opall::SolverSummary &solverSummary,
                                         std::optional<std::reference_wrapper<const opall::ResidualContainer>> residuals = std::nullopt)
    : m_optimizationDataContainer(optimizationDataContainer), m_solverSummary(solverSummary)
{
}

const opall::optimization_data_container::OptimizationDataContainer &opall_solver_app::ReportData::getOptimizationDataContainer() const
{
    return m_optimizationDataContainer;
}

const opall::SolverSummary &opall_solver_app::ReportData::getSolverSummary() const
{
    return m_solverSummary;
}

const std::optional<std::reference_wrapper<const opall::ResidualContainer>> &opall_solver_app::ReportData::getResiduals() const
{
    return m_residuals;
}
