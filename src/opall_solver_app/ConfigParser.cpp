#include "ConfigParser.hpp"

#include <fstream>
#include <print>
#include <ranges>
#include <set>
#include <sstream>
#include <algorithm>
// #include <iostream> //TODO: remove if/when not needed

// TODO: consider refactoring this class so it can be re-used in other apps using opall solver. Consider moving it to library.
namespace opall_solver_app
{

std::vector<std::string> ConfigParser::ArgumentParser::splitLine(const std::string &line) const
{
    std::istringstream iss(line);
    std::string word;
    auto parsedStrings{std::vector<std::string>{}};
    while (iss >> word)
    {
        if (word.at(0) == '#') // TODO: add comment char somwehere as constexpr
        {
            break;
        }
        parsedStrings.push_back(word);
    }
    return parsedStrings;
}

ConfigParser::ArgumentParser::ParsedResult ConfigParser::ArgumentParser::parseString(const std::string &line,
                                                                                     [[maybe_unused]] std::ifstream &file) const
{
    const auto parsedStrings{splitLine(line)};

    if (parsedStrings.size() < 2)
    {
        return std::unexpected("Error while parsing string value from config in line: '"s + line + "'"s);
    }

    return parsedStrings.at(1);
}

ConfigParser::ArgumentParser::ParsedResult ConfigParser::ArgumentParser::parseBool(const std::string &line,
                                                                                   [[maybe_unused]] std::ifstream &file) const
{
    const auto parsedStrings{splitLine(line)};

    if (parsedStrings.size() < 2)
    {
        return std::unexpected("Error while parsing string value from config in line: '"s + line + "'"s);
    }

    if (parsedStrings[1] == "true"s)
    {
        return true;
    }

    if (parsedStrings[1] == "false"s)
    {
        return false;
    }

    return std::unexpected("Error while parsing string value from config in line: '"s + line + "' Expected true of false string."s);
}

ConfigParser::ArgumentParser::ParsedResult ConfigParser::ArgumentParser::parseDoublePositive(const std::string &line,
                                                                                             [[maybe_unused]] std::ifstream &file) const
{
    const auto parsedStrings{splitLine(line)};

    if (parsedStrings.size() < 2)
    {
        return std::unexpected("Error while parsing string value from config in line: '"s + line + "'"s);
    }

    const double val{std::atof(parsedStrings[1].c_str())};

    if (val <= 0.0)
    {
        return std::unexpected("Error while parsing string value from config in line: '"s + line + "' Expected positive double."s);
    }

    return val;
}

ConfigParser::ArgumentParser::ParsedResult ConfigParser::ArgumentParser::parseLossFunctionData([[maybe_unused]] const std::string &line,
                                                                                               std::ifstream &file) const
{
    auto parsedResult{ParsedResult{}};
    std::string lineWithLossFunctionInformation;

    if (std::getline(file, lineWithLossFunctionInformation, '\n'))
    {
        const auto lossFuntionTypeParseResult{parseString(lineWithLossFunctionInformation, file)};
        if (!lossFuntionTypeParseResult.has_value())
        {
            std::print("Error while parsing name of a loss function.\n");
            return lossFuntionTypeParseResult;
        }
        const auto lossFuntionType{*std::get_if<std::string>(&lossFuntionTypeParseResult.value())};

        if (lossFuntionType != "TRIVIAL")
        {
            if (std::getline(file, lineWithLossFunctionInformation, '\n'))
            {
                const auto lossFuntionTypeParseResult{parseDoublePositive(lineWithLossFunctionInformation, file)};
                if (!lossFuntionTypeParseResult.has_value())
                {
                    std::print("Error while parsing scale parameter of a loss function.\n");
                    return lossFuntionTypeParseResult;
                }
                const auto lossFuntionParameter{*std::get_if<double>(&lossFuntionTypeParseResult.value())};

                if (lossFuntionType == "CAUCHY")
                {
                    return opall::CauchyLoss{lossFuntionParameter};
                }

                if (lossFuntionType == "HUBER")
                {
                    return opall::HuberLoss{lossFuntionParameter};
                }

                if (lossFuntionType == "TUKEY")
                {
                    return opall::TukeyLoss{lossFuntionParameter};
                }

                return std::unexpected("Unknown loss function type: "s + lossFuntionType);
            }
            else
            {
                return std::unexpected("Error while parsing loss function data."s);
            }
        }
        else
        {
            [[maybe_unused]] std::string lineWithUnusedParameterValue;
            if (std::getline(file, lineWithUnusedParameterValue, '\n'))
            {
                return opall::TrivialLoss{};
            }
            else
            {
                return std::unexpected("Error while parsing loss function data."s);
            }
        }
    }
    else
    {
        return std::unexpected("Error while parsing loss function data"s);
    }
    return parsedResult;
}

ConfigParser::ConfigParser()
{
}

opall_solver_app::Config ConfigParser::parseFile(std::filesystem::path path)
{
    //std::println("path to config file: {}", path.string());
    
    if (!std::filesystem::exists(path))
    {
        throw std::invalid_argument("Fatal Error. Config file does not esist!");
    }

    parsedParameters.clear();
    auto file{std::ifstream{std::move(path)}};

    for (std::string line; std::getline(file, line, '\n');)
    {
        // std::cout << "reading line: " << line << std::endl;
        const auto paramName{line | std::views::drop_while([](char c) { return std::isspace(c); }) |
                             std::views::take_while([](char c) { return c != '#' && !std::isspace(c); }) | std::ranges::to<std::string>()};
        // std::cout << "paramName: '" << paramName << "'" << std::endl;
        if (paramName.empty())
        {
            continue;
        }

        if (paramName.back() == ':') [[likely]]
        {
            if (!legalParams.contains(paramName))
            {
                throw std::runtime_error("Ilegal config parameter name: "s + paramName);
            }
        }
        else [[unlikely]]
        {
            continue;
        }

        if (!actions.contains(paramName))
        {
            continue;
        }

        // std::cout << "parsing " << paramName << std::endl;
        const auto parsedResult{actions.at(paramName)(argParser, line, file)};

        if (!parsedResult.has_value())
        {
            throw std::runtime_error(parsedResult.error());
        }

        parsedParameters.insert({paramName, parsedResult.value()});
    }

    checkIfAllParametersWereParsed();

    const auto config{createConfigUsingParsedParameters()};
    return config;
}

void ConfigParser::checkIfAllParametersWereParsed() const
{
    const auto namesOfParsedParameters{parsedParameters | std::views::keys | std::ranges::to<std::set>()};
    const auto namesOfRequiredParameters{actions | std::views::keys | std::ranges::to<std::set>()};

    std::vector<std::string> missingParameters;
    std::ranges::set_difference(namesOfRequiredParameters, namesOfParsedParameters, std::back_inserter(missingParameters));

    if (!missingParameters.empty())
    {
        std::print("Following parameters in the config file were not specified:\n");
        for (const auto &paramName : missingParameters)
        {
            std::print("'{}'\n", paramName);
        }
        throw std::runtime_error("Missing config parameters."s);
    }
}

Config ConfigParser::createConfigUsingParsedParameters() const
{
    auto config{opall_solver_app::Config{}};

    const auto costFunctionType{std::get<std::string>(parsedParameters.at("cost_function_type:"s))};
    if (!stringToCorrespondenceTypeMap.contains(costFunctionType))
    {
        throw std::runtime_error("Error while parsing config file. Specified cost function type: '" + costFunctionType + "' is not valid."s);
    }
    config.pointIn3DCorrespondenceType = stringToCorrespondenceTypeMap.at(costFunctionType);

    config.modellingConfig.lossFunctionOfObservedPoint =
        std::get<opall::LossFunctionDescription>(parsedParameters.at("observed_point_loss_function:"s));
    config.modellingConfig.lossFunctionOfPointToPoint =
        std::get<opall::LossFunctionDescription>(parsedParameters.at("point_to_point_loss_function:"s));

    const auto sparseLinearAlgebraEngine{std::get<std::string>(parsedParameters.at("sparse_algebra_engine:"s))};
    if (!stringToSparseAlgebraEngineMap.contains(sparseLinearAlgebraEngine))
    {
        throw std::runtime_error("Error while parsing config file. Specified sparse linear algebra engine: '" + sparseLinearAlgebraEngine +
                                 "' is not valid."s);
    }
    config.optimizationConfig.sparseAlgebraEngine = stringToSparseAlgebraEngineMap.at(sparseLinearAlgebraEngine);

    const auto linearSolverType{std::get<std::string>(parsedParameters.at("linear_solver_type:"s))};
    if (!stringToLinearSolverTypeMap.contains(linearSolverType))
    {
        throw std::runtime_error("Error while parsing config file. Specified linear solver type: '"s + linearSolverType + "' is not valid."s);
    }
    config.optimizationConfig.linearSolverType = stringToLinearSolverTypeMap.at(linearSolverType);

    config.optimizationConfig.printOptimizationSteps = std::get<bool>(parsedParameters.at("print_optimization_steps:"s));
    config.optimizationConfig.printSolverReport = std::get<bool>(parsedParameters.at("print_solver_report:"s));

    config.reportConfig.printResiduals = std::get<bool>(parsedParameters.at("print_residuals:"s));
    config.reportConfig.printLandmarks = std::get<bool>(parsedParameters.at("print_optimized_landmarks:"s));

    return config;
}

} // namespace opall_solver_app