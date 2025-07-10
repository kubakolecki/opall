#include "CommandLineParameters.hpp"
#include "ConfigParser.hpp"
#include "ReportData.hpp"
#include "input_output.hpp"

#include <opall/IncrementalAlignment.hpp>
#include <opall/OptimizationDataContainer.hpp>
#include <opall/Problem.hpp>
#include <opall/cost_function_data_creator.hpp>
#include <opall/fill_problem.hpp>
#include <opall/optimization_data_processor.hpp>
#include <opall/residual_computation.hpp>
#include <opall/full_covariance_computation.hpp>
#include <opall/block_covariance_computation.hpp>

#include <filesystem>
#include <fstream>
#include <print>
#include <chrono> //temporary for profiling
#include <iostream>

using namespace std::literals;

static constexpr std::string_view appName{"Opall Solver App"sv};

void preprocessCommandLineParameters(const opall_solver_app::CommandLineParameters &parameters);
bool checkIfContinueProcessing(const opall_solver_app::CommandLineParameters &parameters);
void printIncrementalAlignmentStatus(const opall::IncrementalAlignmentStatus &status);

int main(int argc, char **argv)
{
    opall_solver_app::CommandLineParameters commandLineParameters{appName};

    try
    {
        std::print("Running {}\n", appName);
        commandLineParameters.parseParameters(argc, argv);
        preprocessCommandLineParameters(commandLineParameters);
        if (!checkIfContinueProcessing(commandLineParameters))
        {
            return EXIT_SUCCESS;
        }
        opall_solver_app::ConfigParser configParser;
        const auto config{configParser.parseFile(commandLineParameters.getValueOrEmptyString("config"))};
        opall::optimization_data_container::OptimizationDataContainer optimizationDataContainer;
        optimizationDataContainer.pointObservationContainer =
            opall_solver_app::readPointObservations(commandLineParameters.getValueOrEmptyString("measurements"s));
        if (commandLineParameters.wasLoaded("poses"s))
        {
            optimizationDataContainer.posesContainer = opall_solver_app::readPoses(commandLineParameters.getValueOrEmptyString("poses"s));
            if (opall::optimization_data_container::checkConsistency(optimizationDataContainer) !=
                opall::optimization_data_container::ConsistencyEvaluationResult::SUCCESS)
            {
                throw std::logic_error("Optimization data container is inconsistent.");
            }
        }
        else
        {
            opall::IncrementalAlignment incrementalAlignment{config.modellingConfig, config.optimizationConfig};
            auto [alignedDataContaier, alignemtStatus]{incrementalAlignment.align(optimizationDataContainer)};
            printIncrementalAlignmentStatus(alignemtStatus);
            optimizationDataContainer = std::move(alignedDataContaier);
        }

        optimizationDataContainer.pointContainer =
            opall::computePointCoordinates(optimizationDataContainer.pointObservationContainer, optimizationDataContainer.posesContainer);

        opall::Problem optimizationProblem;

        // TODO: make the strategy a variant type
        const auto costFunctionDataContainer{opall::cost_function_data::createCostFunctionData(
            optimizationDataContainer, config.modellingConfig, opall::cost_function_data::ObservedPointIn3DStrategy{})};
        std::print("adding const functions to the problem:\n");

        opall::fillProblem(costFunctionDataContainer,
                           optimizationProblem); // TODO: move fill problem free function to Problem.cpp file and get rid of fill_problem files.
        std::print("solving ceres problem:\n");
        const auto solverSummary{optimizationProblem.solve(config.optimizationConfig)};

        std::optional<opall::ResidualContainer> residuals{std::nullopt};

        if (config.reportConfig.printResiduals)
        {
            std::print("computing residuals:\n");
            residuals = opall::computeResiduals(costFunctionDataContainer);
            std::print("residuals has value: {}\n",residuals.has_value());
        }

        opall_solver_app::ReportData reportData{optimizationDataContainer, solverSummary, residuals};
        opall_solver_app::printReport(std::filesystem::path(commandLineParameters.getValueOrEmptyString("output"s)) /
                                          std::filesystem::path("report.txt"s),
                                      reportData, config.reportConfig);

        const auto jacobianData {optimizationProblem.getJacobian()};
        
        Eigen::SparseMatrix<double> sparseJacobian{jacobianData.numberOfRows, jacobianData.numberOfColumns};
        sparseJacobian.setFromTriplets(jacobianData.triplets.begin(), jacobianData.triplets.end());

        opall_solver_app::printMatrix(std::filesystem::path(commandLineParameters.getValueOrEmptyString("output"s)) /
                                          std::filesystem::path("jacobian.txt"s), sparseJacobian, 2);

        sparseJacobian.makeCompressed();
        Eigen::SparseMatrix<double>  hessian {sparseJacobian.transpose()*sparseJacobian};


        opall_solver_app::printMatrix(std::filesystem::path(commandLineParameters.getValueOrEmptyString("output"s)) /
                                          std::filesystem::path("hessian.txt"s), hessian, 2);


        const auto timeStart {std::chrono::high_resolution_clock::now()};                                        
        const Eigen::MatrixXd covariance = opall::covariance::computeFullCovariance(jacobianData, opall::covariance::computeUsingNaiveMatrixInversion);
        const auto timeEnd  {std::chrono::high_resolution_clock::now()};
        
        std::chrono::duration<double, std::milli> duration {timeEnd - timeStart};
        std::print("duration of covariance matrix computation [ms]: {}\n", duration);

        opall::SparseJacobianData sandboxData;

        sandboxData.numberOfColumns = 4;
        sandboxData.numberOfRows = 6;

        sandboxData.triplets.push_back({0,0,1.0});
        sandboxData.triplets.push_back({1,0,1.0});
        sandboxData.triplets.push_back({2,0,1.0});
        sandboxData.triplets.push_back({3,0,1.0});
        sandboxData.triplets.push_back({4,0,1.0});
        sandboxData.triplets.push_back({5,0,1.0});

        sandboxData.triplets.push_back({3,1,1.0});
        sandboxData.triplets.push_back({4,1,1.0});
        sandboxData.triplets.push_back({5,1,1.0});

        sandboxData.triplets.push_back({1,2,1.0});
        sandboxData.triplets.push_back({2,2,2.0});

        sandboxData.triplets.push_back({4,2,1.0});
        sandboxData.triplets.push_back({5,2,2.0});

        sandboxData.triplets.push_back({4,3,1.0});
        sandboxData.triplets.push_back({5,3,2.0});

        //const auto timeStartCovarianceFast {std::chrono::high_resolution_clock::now()};                                        
        //const Eigen::MatrixXd covarianceFast = opall::covariance::computeFullCovariance(jacobianData, opall::covariance::computeUsingApproximateMatrixInversionEigen);
        //const auto timeEndCovarianceFast  {std::chrono::high_resolution_clock::now()};

        std::print("testing on sandbox data\n");
        const auto timeStartCovarianceFast {std::chrono::high_resolution_clock::now()};                                        
        const Eigen::MatrixXd covarianceFast = opall::covariance::computeFullCovariance(sandboxData, opall::covariance::computeUsingApproximateMatrixInversionEigen);
        const auto timeEndCovarianceFast  {std::chrono::high_resolution_clock::now()};
        
        std::chrono::duration<double, std::milli> durationCovarianceFast {timeEndCovarianceFast - timeStartCovarianceFast};
        std::print("duration of fast covariance matrix computation [ms]: {}\n", durationCovarianceFast);


        const auto parameterBlockDataContainer{optimizationProblem.getParameterBlockData()};

        //std::print("parameter block data:\n");
        //for (const auto& blockData: parameterBlockData.data)
        //{
        //    std::print("{} {} {}\n", static_cast<const void*>(blockData.address), blockData.position, blockData.size);
        //}

        //checking if addresses of poses are there:
        
        opall::covariance::BlockMatrixDataContainer covarianceSubmatricesData;

        for (const auto&[poseId, pose]:  optimizationDataContainer.posesContainer)
        {
            if (pose.type == opall::Pose::Type::FIXED)
            {
                continue;
            }

            const auto paramBlockDataIt = std::ranges::find_if(parameterBlockDataContainer.data, [&](const auto& parameterBlockData){return pose.position.data() == parameterBlockData.address; }  );

            std::print("searching for parameter block data of pose {}\n", poseId);
            if (paramBlockDataIt == parameterBlockDataContainer.data.end() )
            {
                std::print("parameter block data not found\n");
            }
            else
            {
                std::print("Found following parameter block data:\n");
                std::print("{} {} {}\n", static_cast<const void*>((*paramBlockDataIt).address), (*paramBlockDataIt).position, (*paramBlockDataIt).size);
                covarianceSubmatricesData.addData((*paramBlockDataIt).position, (*paramBlockDataIt).position, (*paramBlockDataIt).size, (*paramBlockDataIt).size );
            }
        }

        const auto covarianceMatricesForPosition = opall::covariance::getSubmatrices(covarianceSubmatricesData, covariance);
        //const auto covarianceMatricesForPositionFast = opall::covariance::getSubmatrices(covarianceSubmatricesData, covarianceFast);



        std::cout <<"covariance matrices from full covariance:\n";
        for (const auto& covMatrix: covarianceMatricesForPosition)
        {
            std::cout <<"covariance matrix:\n";
            if (covMatrix.has_value() )
            {
                std::cout << covMatrix.value() << "\n";
            }
        }

        //std::cout <<"covariance matrices from full fast covariance:\n";
        //for (const auto& covMatrix: covarianceMatricesForPositionFast)
        //{
        //    std::cout <<"covariance matrix:\n";
        //    if (covMatrix.has_value() )
        //    {
        //        std::cout << covMatrix.value() << "\n";
        //    }
        //}

        opall::Problem::CovarianceBlockData covarianceBlockData;
        covarianceBlockData.parameterBlockAddresses.reserve(optimizationDataContainer.posesContainer.size());
        covarianceBlockData.parameterBlockSizes.reserve(optimizationDataContainer.posesContainer.size());
        for (const auto&[poseId, pose]:  optimizationDataContainer.posesContainer)
        {
            if (pose.type == opall::Pose::Type::FIXED)
            {
                continue;
            }
            covarianceBlockData.parameterBlockAddresses.emplace_back(pose.position.data(),pose.position.data());
            covarianceBlockData.parameterBlockSizes.emplace_back(3,3);
            covarianceBlockData.parameterBlockAddresses.emplace_back(pose.quaternionWxyz.data(), pose.quaternionWxyz.data());
            covarianceBlockData.parameterBlockSizes.emplace_back(3,3);

        }

        const auto timeStartCovProblem {std::chrono::high_resolution_clock::now()};
        const auto covarianceDataComputedFromProblem = optimizationProblem.computeCovarianceMatrices(covarianceBlockData);
        const auto timeEndCovProblem {std::chrono::high_resolution_clock::now()};

        std::chrono::duration<double, std::milli> durationCovProblem {timeEndCovProblem - timeStartCovProblem};
        std::print("duration of covariance matrix computation [ms]: {}\n", durationCovProblem);

        std::cout <<"covariance matrices from full problem object:\n";
        if (covarianceDataComputedFromProblem.has_value())
        {     
            for (const auto& covMatrix : covarianceDataComputedFromProblem.value())
            {
                std::cout <<"covariance matrix:\n";
                std::cout << covMatrix <<"\n";
            }
        }
        else
        {
            std::print("Covariance computation using problem object failed.");
        }

        
        opall_solver_app::printMatrix(std::filesystem::path(commandLineParameters.getValueOrEmptyString("output"s)) /
                                          std::filesystem::path("covariance.txt"s), covariance, 2);

    }
    catch (std::exception e)
    {
        std::print("{:s}\n", e.what());
        std::print("Exiting with fatal error.\n");
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;

    // OPALL: Open Pointcloud ALignement Library
}

void preprocessCommandLineParameters(const opall_solver_app::CommandLineParameters &parameters)
{
    if (parameters.wasLoaded("version"))
    {
        std::print("{} version: {}.{}.{}\n", appName, APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_PATCH);
    }

    if (parameters.wasLoaded("help"))
    {
        parameters.printHelp();
    }

    if (parameters.wasLoaded("output"))
    {
        const auto outputDirectory{std::filesystem::path(parameters.getValueOrEmptyString("output"))};

        if (!std::filesystem::exists(outputDirectory))
        {
            throw std::runtime_error("Fatal Error! Output directory: " + outputDirectory.string() + " does not exist!\n");
        }

        if (std::filesystem::directory_iterator(outputDirectory) != std::filesystem::directory_iterator{})
        {
            if (!parameters.wasLoaded("overwrite"))
            {
                throw std::runtime_error("Fatal Error! Output directory is not empty! Use " + parameters.getParameter("overwrite").shortPrompt + "/" +
                                         parameters.getParameter("overwrite").longPrompt +
                                         " command line argument to explicitly allow for data overwriting.\n");
            }
        }
    }
}

bool checkIfContinueProcessing(const opall_solver_app::CommandLineParameters &parameters)
{
    if (parameters.wasLoaded("output") && parameters.wasLoaded("measurements") && parameters.wasLoaded("config"))
    {
        return true;
    }

    return false;
}

void printIncrementalAlignmentStatus(const opall::IncrementalAlignmentStatus &status)
{
    switch (status)
    {
    case opall::IncrementalAlignmentStatus::TO_FEW_CORRESPONDENCES:
        std::print("Incremental alignment failed, because one pose contains too few correspondences. Not all poses were aligned.\n");
        break;
    case opall::IncrementalAlignmentStatus::LEAST_SQUARE_SOLVER_FAILED:
        std::print("Incremental alignment failed, because least square solver failed for certain pose. Not all poses were aligned.\n");
        break;
    case opall::IncrementalAlignmentStatus::SUCESS:
        std::print("Incremental alignemnt was sucessfull.\n");
        break;
    default:
        break;
    }
}
