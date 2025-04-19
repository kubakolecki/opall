#include "opall/Problem.hpp"
#include "opall/LossFunctionDescription.hpp"
#include "opall/SolverSummary.hpp"
#include "cost_functors/ObservedPointIn3DCostFunctor.hpp"
#include "cost_functors/ObservedPointIn3DFixedStationCostFunctor.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <ceres/problem.h>

#include <print>
#include <unordered_map>
#include <variant>

class opall::Problem::Impl
{
  public:
    void insertToProblem(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData);
    void insertToProblem(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData);
    opall::SolverSummary solve(const OptimizationConfig &config);

  private:
    template <class... Ts>
    struct OverloadLossFunctionCreator : Ts...
    {
        using Ts::operator()...;
    };

    ceres::Solver::Options createCeresSolverOptions(const OptimizationConfig &config);
    ceres::LossFunction *createLossFunction(const LossFunctionDescription &lossFunctionDescription) const;
    opall::SolverSummary createSolverSummary(const ceres::Solver::Summary &ceresSummary) const;
    ceres::Problem ceresProblem;

    std::unordered_map<OptimizationConfig::LinearSolverType, ceres::LinearSolverType> linearSolverTypeMap{
        {OptimizationConfig::LinearSolverType::CGNR, ceres::LinearSolverType::CGNR},
        {OptimizationConfig::LinearSolverType::DENSE_NORMAL_CHOLESKY, ceres::LinearSolverType::DENSE_NORMAL_CHOLESKY},
        {OptimizationConfig::LinearSolverType::SPARSE_NORMAL_CHOLESKY, ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY},
        {OptimizationConfig::LinearSolverType::DENSE_QR, ceres::LinearSolverType::DENSE_QR},
        {OptimizationConfig::LinearSolverType::DENSE_SCHUR, ceres::LinearSolverType::DENSE_SCHUR},
        {OptimizationConfig::LinearSolverType::SPARSE_SCHUR, ceres::LinearSolverType::SPARSE_SCHUR},
        {OptimizationConfig::LinearSolverType::ITERATIVE_SCHUR, ceres::LinearSolverType::ITERATIVE_SCHUR}};

    std::unordered_map<OptimizationConfig::SparseAlgebraEngine, ceres::SparseLinearAlgebraLibraryType> sparseAlgebraEngineMap{
        {OptimizationConfig::SparseAlgebraEngine::EIGEN_SPARSE, ceres::SparseLinearAlgebraLibraryType::EIGEN_SPARSE},
        {OptimizationConfig::SparseAlgebraEngine::SUIT_SPARSE, ceres::SparseLinearAlgebraLibraryType::SUITE_SPARSE}};
};

void opall::Problem::Impl::insertToProblem(
    const opall::cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData)
{
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<cost_functors::ObservedPointIn3DFixedStationCostFunctor, 3, 3>(
        new cost_functors::ObservedPointIn3DFixedStationCostFunctor(costFunctionData.observedPoint, costFunctionData.pose.position,
                                                                    costFunctionData.pose.quaternionWxyz, costFunctionData.uncertainty));
    ceres::LossFunction *lossFunction{createLossFunction(costFunctionData.lossFunctionDescription)};

    ceresProblem.AddResidualBlock(costFunction, lossFunction, costFunctionData.point);
}

void opall::Problem::Impl::insertToProblem(const opall::cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData)
{
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<cost_functors::ObservedPointIn3DCostFunctor, 3, 3, 4, 3>(
        new cost_functors::ObservedPointIn3DCostFunctor(costFunctionData.observedPoint, costFunctionData.uncertainty));

    ceres::LossFunction *lossFunction{createLossFunction(costFunctionData.lossFunctionDescription)};

    ceresProblem.AddResidualBlock(costFunction, lossFunction, costFunctionData.posePosition, costFunctionData.poseWxyzQuaternion,
                                  costFunctionData.point);
    ceresProblem.AddParameterBlock(costFunctionData.poseWxyzQuaternion, 4, new ceres::QuaternionManifold{});
}

opall::SolverSummary opall::Problem::Impl::solve(const OptimizationConfig &config)
{
    ceres::Solver::Summary ceresSummary;
    ceres::Solve(createCeresSolverOptions(config), &ceresProblem, &ceresSummary);

    if (config.printSolverReport)
    {
        std::print("{}\n", ceresSummary.FullReport());
    }

    const auto solverSummary{createSolverSummary(ceresSummary)};
    return solverSummary;
}

ceres::Solver::Options opall::Problem::Impl::createCeresSolverOptions(const OptimizationConfig &config)
{
    auto options{ceres::Solver::Options{}};

    options.sparse_linear_algebra_library_type = sparseAlgebraEngineMap.at(config.sparseAlgebraEngine);
    options.linear_solver_type = linearSolverTypeMap.at(config.linearSolverType);
    options.minimizer_progress_to_stdout = config.printOptimizationSteps;

    return options;
}

ceres::LossFunction *opall::Problem::Impl::createLossFunction(const LossFunctionDescription &lossFunctionDescription) const
{
    ceres::LossFunction *lossFcnPtr = std::visit(OverloadLossFunctionCreator{
                                                     [](TrivialLoss loss) {
                                                         ceres::LossFunction *lFPtr = new ceres::TrivialLoss{};
                                                         return lFPtr;
                                                     },
                                                     [](CauchyLoss loss) {
                                                         ceres::LossFunction *lFPtr = new ceres::CauchyLoss{loss.parameter};
                                                         return lFPtr;
                                                     },
                                                     [](HuberLoss loss) {
                                                         ceres::LossFunction *lFPtr = new ceres::HuberLoss{loss.parameter};
                                                         return lFPtr;
                                                     },
                                                     [](TukeyLoss loss) {
                                                         ceres::LossFunction *lFPtr = new ceres::TukeyLoss{loss.parameter};
                                                         return lFPtr;
                                                     },
                                                 },
                                                 lossFunctionDescription);
    return lossFcnPtr;
}

opall::SolverSummary opall::Problem::Impl::createSolverSummary(const ceres::Solver::Summary &ceresSummary) const
{
    opall::SolverSummary summary;

    summary.fullReport = ceresSummary.FullReport();
    summary.isSolutionUsable = ceresSummary.IsSolutionUsable();
    summary.initialCost = ceresSummary.initial_cost;
    summary.finalCost = ceresSummary.final_cost;
    summary.totalTimeInSeconds = ceresSummary.total_time_in_seconds;
    summary.numOfParameterBlocks = ceresSummary.num_parameter_blocks;
    summary.numOfParameters = ceresSummary.num_parameters;
    summary.numOfEffectiveParameters = ceresSummary.num_effective_parameters;
    summary.numOfResidualBlocks = ceresSummary.num_residual_blocks;
    summary.numOfResiduals = ceresSummary.num_residuals;
    summary.numOfThreadsUsed = ceresSummary.num_threads_used;
    summary.sigmaZero = std::sqrt(2.0 * summary.finalCost / (summary.numOfResiduals - summary.numOfEffectiveParameters));

    return summary;
}

opall::Problem::Problem() : pimpl{std::make_unique<Impl>()}
{
}

opall::Problem::~Problem() = default;

opall::Problem::Problem(opall::Problem &&other) noexcept : pimpl{std::make_unique<Impl>(std::move(*other.pimpl))}
{
}

opall::Problem &opall::Problem::operator=(opall::Problem &&other) noexcept
{
    *pimpl = std::move(*other.pimpl);
    return *this;
}

void opall::Problem::insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData)
{
    pimpl->insertToProblem(costFunctionData);
}

void opall::Problem::insertCostFunctionToProblem(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData)
{
    pimpl->insertToProblem(costFunctionData);
}

opall::SolverSummary opall::Problem::solve(const OptimizationConfig &config)
{
    const auto solverSummary{pimpl->solve(config)};
    return solverSummary;
}
