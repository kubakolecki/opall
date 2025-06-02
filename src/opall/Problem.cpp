#include "opall/Problem.hpp"
#include "opall/LossFunctionDescription.hpp"
#include "opall/SolverSummary.hpp"
#include "cost_functors/ObservedPointIn3DCostFunctor.hpp"
#include "cost_functors/ObservedPointIn3DFixedStationCostFunctor.hpp"

#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <ceres/problem.h>

#include <Eigen/Sparse>

#include <print>
#include <unordered_map>
#include <variant>

class opall::Problem::Impl
{
  public:
    Impl();
    void insertToProblem(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData);
    void insertToProblem(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData);
    opall::SolverSummary solve(const OptimizationConfig &config);
    opall::SparseJacobianData getJacobian();
    opall::ParameterBlockDataContainer getParameterBlockData() const;
    std::expected<opall::Problem::CovarianceComputationResult, opall::Problem::CovarianceComputationError> computeCovarianceMatrices(const CovarianceBlockData& covarianceBlockData);
    

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
    std::vector<double*> parametersInOrderOfInsertions;

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

opall::Problem::Impl::Impl()
{
    parametersInOrderOfInsertions.reserve(8192uz);
}

void opall::Problem::Impl::insertToProblem(
    const opall::cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData)
{
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<cost_functors::ObservedPointIn3DFixedStationCostFunctor, 3, 3>(
        new cost_functors::ObservedPointIn3DFixedStationCostFunctor(costFunctionData.observedPoint, costFunctionData.pose.position,
                                                                    costFunctionData.pose.quaternionWxyz, costFunctionData.uncertainty));
    ceres::LossFunction *lossFunction{createLossFunction(costFunctionData.lossFunctionDescription)};

    ceresProblem.AddResidualBlock(costFunction, lossFunction, costFunctionData.point);

    parametersInOrderOfInsertions.push_back(costFunctionData.point);
}

void opall::Problem::Impl::insertToProblem(const opall::cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData)
{
    ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<cost_functors::ObservedPointIn3DCostFunctor, 3, 3, 4, 3>(
        new cost_functors::ObservedPointIn3DCostFunctor(costFunctionData.observedPoint, costFunctionData.uncertainty));

    ceres::LossFunction *lossFunction{createLossFunction(costFunctionData.lossFunctionDescription)};

    ceresProblem.AddResidualBlock(costFunction, lossFunction, costFunctionData.posePosition, costFunctionData.poseWxyzQuaternion,
                                  costFunctionData.point);
    ceresProblem.AddParameterBlock(costFunctionData.poseWxyzQuaternion, 4, new ceres::QuaternionManifold{});

    parametersInOrderOfInsertions.push_back(costFunctionData.posePosition);
    parametersInOrderOfInsertions.push_back(costFunctionData.poseWxyzQuaternion);
    parametersInOrderOfInsertions.push_back(costFunctionData.point);
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

opall::SparseJacobianData opall::Problem::Impl::getJacobian()
{
    ceres::CRSMatrix jacobian;
    auto evaluateOptions {ceres::Problem::EvaluateOptions{}};
    evaluateOptions.apply_loss_function = false;
    ceresProblem.Evaluate(evaluateOptions, nullptr, nullptr, nullptr, &jacobian);


    SparseJacobianData jacobianData;
    jacobianData.numberOfColumns = jacobian.num_cols;
    jacobianData.numberOfRows = jacobian.num_rows;
    jacobianData.triplets.reserve(jacobian.values.size());

    for (auto row {0}; row < jacobian.num_rows; ++row) 
    {
        auto start {jacobian.rows[row]};
        auto end {jacobian.rows[row + 1]};
        for (auto idx{start}; idx < end; ++idx) 
        {
            auto col {jacobian.cols[idx]};
            double val {jacobian.values[idx]};
            jacobianData.triplets.emplace_back(row, col, val);
        }
    }

    return jacobianData;
}

opall::ParameterBlockDataContainer opall::Problem::Impl::getParameterBlockData() const
{
    std::vector<double*> parametersInOrderOfInsertionUnique;
    parametersInOrderOfInsertionUnique.reserve(parametersInOrderOfInsertions.size());
    std::unordered_set<double*> checkedAddresses;
    
    for (const auto parameterBlockAddress: parametersInOrderOfInsertions)
    {
        if (!checkedAddresses.contains(parameterBlockAddress) )
        {
            parametersInOrderOfInsertionUnique.push_back(parameterBlockAddress);
        }
        checkedAddresses.insert(parameterBlockAddress);
    }
    parametersInOrderOfInsertionUnique.shrink_to_fit();
    
    //std::vector<double*> parameterBlockAddresses; 
    //ceresProblem.GetParameterBlocks(&parameterBlockAddresses);
    //const auto numberOfParameterBlockData{parameterBlockAddresses.size()};
    const auto numberOfParameterBlockData{parametersInOrderOfInsertionUnique.size()};
    
    std::vector<int> parameterBlockSizes;
    parameterBlockSizes.reserve(numberOfParameterBlockData);
    //std::ranges::transform(parameterBlockAddresses,std::back_inserter(parameterBlockSizes), [this](double* address){  return ceresProblem.ParameterBlockTangentSize(address); } );
    std::ranges::transform(parametersInOrderOfInsertionUnique,std::back_inserter(parameterBlockSizes), [this](double* address){  return ceresProblem.ParameterBlockTangentSize(address); } );
    std::vector<int> positions;
    positions.reserve(numberOfParameterBlockData);
    std::exclusive_scan(parameterBlockSizes.begin(), parameterBlockSizes.end(), std::back_inserter(positions), 0, std::plus<>{} );
    
    auto parameterBlockData{opall::ParameterBlockDataContainer{}};
    parameterBlockData.data.reserve(numberOfParameterBlockData);

    for (auto paramId{0uz}; paramId< numberOfParameterBlockData; ++paramId )
    {
        //parameterBlockData.data.emplace_back(parameterBlockAddresses[paramId], positions[paramId], parameterBlockSizes[paramId]);
        parameterBlockData.data.emplace_back(parametersInOrderOfInsertionUnique[paramId], positions[paramId], parameterBlockSizes[paramId]);
    }

    return parameterBlockData;
}


std::expected<opall::Problem::CovarianceComputationResult, opall::Problem::CovarianceComputationError> opall::Problem::Impl::computeCovarianceMatrices(const CovarianceBlockData& covarianceBlockData)
{
    if (covarianceBlockData.parameterBlockAddresses.size() != covarianceBlockData.parameterBlockSizes.size() )
    {
        return std::unexpected(opall::Problem::CovarianceComputationError::INVALID_INPUT);
    }

    const auto sizesAreCorrect = std::ranges::all_of(covarianceBlockData.parameterBlockSizes, [](const auto& sizes){ return sizes.first >= 0uz && sizes.second >= 0uz; } );

    if (!sizesAreCorrect)
    {
        return std::unexpected(opall::Problem::CovarianceComputationError::INVALID_INPUT);
    }
    
    ceres::Covariance::Options ceresCovarianceOptions{};
    ceres::Covariance covarianceEstimator{ceresCovarianceOptions};
    const auto isComputationSuccessfull{covarianceEstimator.Compute(covarianceBlockData.parameterBlockAddresses, &ceresProblem)};

    if (!isComputationSuccessfull)
    {
        return std::unexpected{opall::Problem::CovarianceComputationError::COMPUTATION_ERROR};
    }

    
    const auto numberOfCovarianceBlocks{covarianceBlockData.parameterBlockAddresses.size()};
    opall::Problem::CovarianceComputationResult covarianceMatrices;
    covarianceMatrices.reserve(numberOfCovarianceBlocks);


    for (auto blockId{0uz}; blockId < numberOfCovarianceBlocks; ++blockId )
    {
        const auto& [rows, columns]{covarianceBlockData.parameterBlockSizes.at(blockId)};
        opall::Problem::ParameterCovarianceMatrix covarianceMatrix(rows, columns);
        const auto isCovarianceExtractionSuccessfull{covarianceEstimator.GetCovarianceBlockInTangentSpace(covarianceBlockData.parameterBlockAddresses.at(blockId).first, covarianceBlockData.parameterBlockAddresses.at(blockId).second, covarianceMatrix.data())};
        if (!isCovarianceExtractionSuccessfull)
        {
            return std::unexpected{opall::Problem::CovarianceComputationError::EXTRACTION_ERROR};
        }
        covarianceMatrices.emplace_back(covarianceMatrix);
    }


    return std::expected<opall::Problem::CovarianceComputationResult, opall::Problem::CovarianceComputationError>{std::in_place_t{}, covarianceMatrices};
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

opall::SparseJacobianData opall::Problem::getJacobian()
{
    return pimpl->getJacobian();
}

opall::ParameterBlockDataContainer opall::Problem::getParameterBlockData() const
{
    return pimpl->getParameterBlockData();
}


std::expected<opall::Problem::CovarianceComputationResult, opall::Problem::CovarianceComputationError> opall::Problem::computeCovarianceMatrices(const opall::Problem::CovarianceBlockData& covarianceBlockData)
{
    return pimpl->computeCovarianceMatrices(covarianceBlockData);

}
