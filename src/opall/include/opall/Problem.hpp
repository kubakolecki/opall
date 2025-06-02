#pragma once

#include "opall/OptimizationConfig.hpp"
#include "opall/SolverSummary.hpp"
#include "opall/SparseJacobianData.hpp"
#include "opall/ParameterBlockData.hpp"
#include "opall/cost_function_data/ObservedPointIn3DCostFunctionData.hpp"
#include "opall/cost_function_data/ObservedPointIn3DFixedStationCostFunctionData.hpp"

#include <memory>
#include <expected>


namespace opall
{

class Problem
{
  public:
    using ParameterCovarianceMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using CovarianceComputationResult = std::vector<ParameterCovarianceMatrix>;
    struct CovarianceBlockData
    {
        std::vector<std::pair<const double*, const double*>> parameterBlockAddresses;
        std::vector<std::pair<int, int>> parameterBlockSizes;
    };

    enum class CovarianceComputationError
    {
        INVALID_INPUT,
        COMPUTATION_ERROR,
        EXTRACTION_ERROR
    };



    Problem();
    ~Problem();

    Problem(Problem const &other) = delete;
    Problem &operator=(Problem const &other) = delete;

    Problem(Problem &&other) noexcept;
    Problem &operator=(Problem &&other) noexcept;

    void insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData);
    void insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData);

    opall::SolverSummary solve(const OptimizationConfig &config = OptimizationConfig{});
    opall::SparseJacobianData getJacobian();
    opall::ParameterBlockDataContainer getParameterBlockData() const;

    std::expected<CovarianceComputationResult, CovarianceComputationError> computeCovarianceMatrices(const CovarianceBlockData& covarianceBlockData);

  private:
    struct Impl;
    std::unique_ptr<Impl> const pimpl;
};

} // namespace opall
