#pragma once

#include "Config.hpp"

#include <expected>
#include <filesystem>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

using namespace std::string_literals;

namespace opall_solver_app
{

class ConfigParser
{
  public:
    ConfigParser();
    opall_solver_app::Config parseFile(std::filesystem::path path);

  private:
    class ArgumentParser
    {
      public:
        using ParsedValue = std::variant<std::string, bool, double, opall::LossFunctionDescription>;
        using ErrorMessage = std::string;
        using ParsedResult = std::expected<ParsedValue, ErrorMessage>;
        ParsedResult parseString(const std::string &line, [[maybe_unused]] std::ifstream &file) const;
        ParsedResult parseBool(const std::string &line, [[maybe_unused]] std::ifstream &file) const;
        ParsedResult parseDoublePositive(const std::string &line, [[maybe_unused]] std::ifstream &file) const;
        ParsedResult parseLossFunctionData([[maybe_unused]] const std::string &line, std::ifstream &file) const;

      private:
        std::vector<std::string> splitLine(const std::string &line) const;
    };

    void checkIfAllParametersWereParsed() const;
    Config createConfigUsingParsedParameters() const;
    ArgumentParser argParser;
    const std::unordered_set<std::string> legalParams{"general_config:"s,
                                                      "cost_function_type:"s,
                                                      "modeling_config:"s,
                                                      "observed_point_loss_function:"s,
                                                      "name:"s,
                                                      "parameter:"s,
                                                      "point_to_point_loss_function:"s,
                                                      "solving_config:"s,
                                                      "sparse_algebra_engine:"s,
                                                      "linear_solver_type:"s,
                                                      "print_optimization_steps:"s,
                                                      "print_solver_report:"s,
                                                      "report_config:"s,
                                                      "print_residuals:"s,
                                                      "print_optimized_landmarks:"s

    };

    const std::unordered_map<std::string, std::function<ArgumentParser::ParsedResult(const ArgumentParser &, std::string line, std::ifstream &file)>>
        actions{{"cost_function_type:"s, std::mem_fn(&ArgumentParser::parseString)},
                {"sparse_algebra_engine:"s, std::mem_fn(&ArgumentParser::parseString)},
                {"linear_solver_type:"s, std::mem_fn(&ArgumentParser::parseString)},
                {"print_optimization_steps:"s, std::mem_fn(&ArgumentParser::parseBool)},
                {"print_solver_report:"s, std::mem_fn(&ArgumentParser::parseBool)},
                {"observed_point_loss_function:"s, std::mem_fn(&ArgumentParser::parseLossFunctionData)},
                {"point_to_point_loss_function:"s, std::mem_fn(&ArgumentParser::parseLossFunctionData)},
                {"print_residuals:"s, std::mem_fn(&ArgumentParser::parseBool)},
                {"print_optimized_landmarks:"s, std::mem_fn(&ArgumentParser::parseBool)}};

    std::unordered_map<std::string, ArgumentParser::ParsedValue> parsedParameters;

    const std::unordered_map<std::string, opall::OptimizationConfig::LinearSolverType> stringToLinearSolverTypeMap{
        {"DENSE_QR"s, opall::OptimizationConfig::LinearSolverType::DENSE_QR},
        {"SPARSE_NORMAL_CHOLESKY"s, opall::OptimizationConfig::LinearSolverType::SPARSE_NORMAL_CHOLESKY},
        {"DENSE_NORMAL_CHOLESKY"s, opall::OptimizationConfig::LinearSolverType::DENSE_NORMAL_CHOLESKY},
        {"DENSE_SCHUR"s, opall::OptimizationConfig::LinearSolverType::DENSE_SCHUR},
        {"SPARSE_SCHUR"s, opall::OptimizationConfig::LinearSolverType::SPARSE_SCHUR},
        {"ITERATIVE_SCHUR"s, opall::OptimizationConfig::LinearSolverType::ITERATIVE_SCHUR},
        {"CGNR"s, opall::OptimizationConfig::LinearSolverType::CGNR}};

    const std::unordered_map<std::string, opall::OptimizationConfig::SparseAlgebraEngine> stringToSparseAlgebraEngineMap{
        {"SUIT_SPARSE"s, opall::OptimizationConfig::SparseAlgebraEngine::SUIT_SPARSE},
        {"EIGEN_SPARSE"s, opall::OptimizationConfig::SparseAlgebraEngine::EIGEN_SPARSE}};

    const std::unordered_map<std::string, Config::PointIn3DCorrespondeceType> stringToCorrespondenceTypeMap{
        {"OBSERVED_POINT"s, opall_solver_app::Config::PointIn3DCorrespondeceType::OBSERVED_POINT},
        {"POINT_TO_POINT"s, opall_solver_app::Config::PointIn3DCorrespondeceType::POINT_TO_POINT}};
};

} // namespace opall_solver_app