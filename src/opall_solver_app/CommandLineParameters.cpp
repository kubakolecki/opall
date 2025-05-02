#include "CommandLineParameters.hpp"

#include <algorithm>
#include <exception>
#include <print>
#include <ranges>
#include <system_error>
#include <vector>

using namespace std::literals;
using namespace std::string_literals;

namespace opall_solver_app
{
CommandLineParameters::CommandLineParameters(const std::string_view &appName) : appName(appName)
{
    static constexpr auto isOblitatory{true};
    static constexpr auto isOptional{false};
    static constexpr auto requriesValue{true};
    static constexpr auto hasNoValue{false};

    addParameter({1, "help"s, "-h"s, "--help"s, isOptional, hasNoValue, std::nullopt, "Prints out help."s});
    addParameter({2, "version"s, "-v"s, "--version"s, isOptional, hasNoValue, std::nullopt, "Prints out app version."s});
    addParameter({3, "config"s, "-c"s, "--config"s, isOptional, requriesValue, std::nullopt, "Path to config file."s});
    addParameter({4, "measurements"s, "-m"s, "--measurements"s, isOptional, requriesValue, std::nullopt, "Path to file with point measurements."s});
    addParameter({5, "poses"s, "-p"s, "--poses"s, isOptional, requriesValue, std::nullopt,
                  "Path to file with approximated poses. If not provided the app will try to solve the alignment sequentially."s});
    addParameter({6, "output"s, "-o"s, "--output"s, isOptional, requriesValue, std::nullopt, "Path to the directory with app output."s});
    addParameter({7, "overwrite"s, "-r"s, "--overwrite"s, isOptional, hasNoValue, std::nullopt,
                  "Explicitly allows to overwrite existing data in the output direcory."s});

    parameters["config"s].needs = {"measurements"s, "output"s};
    parameters["measurements"s].needs = {"config"s, "output"s};
    parameters["poses"s].needs = {"measurements"s, "output"s, "config"s};
    parameters["output"s].needs = {"measurements"s, "config"s};
    parameters["overwrite"s].needs = {"measurements"s, "output"s, "config"s};
}

void CommandLineParameters::parseParameters(int argc, char **argv)
{
    clear();
    //const std::vector<std::string_view> args(argv + 1, argv + argc); //this does not compile in gcc

    std::vector<std::string_view> args;
    for (int i = 1; i < argc; ++i) 
    {
        args.emplace_back(argv[i]);
    }


    if (args.size() < getMinNumberOfEntries())
    {
        throw std::runtime_error("Not enough command line arguments! "s + helpSuggestion);
    }

    if (args.size() > getMaxNumberOfEntries())
    {
        throw std::runtime_error("Too many input command line arguments!\n");
    }

    if (!isPrompt(args[0]))
    {
        throw std::runtime_error("Wrong command line argument specifier at positon 1. "s + helpSuggestion);
    }

    auto argsIt{args.begin()};
    while (argsIt != args.end())
    {
        if (!isPrompt(*argsIt))
        {
            const auto position{argsIt - args.begin()};
            throw std::runtime_error("Wrong command line argument specifier at positon "s + std::to_string(position) + " "s + helpSuggestion);
        }
        const auto prompt{std::string{*argsIt}};
        auto &parameter{getArgumentByPrompt(prompt)};
        if (parameter.needsValue)
        {
            if (argsIt + 1 != args.end()) [[likely]]
            {
                if (isPrompt(*(argsIt + 1))) [[unlikely]]
                {
                    throw std::runtime_error("No valid input provided for "s + prompt + " specifier.\n"s);
                }
                else [[likely]]
                {
                    argsIt++;
                    parameter.value = std::optional<std::string>{*argsIt};
                }
            }
            else [[unlikely]]
            {
                throw std::runtime_error("No input value provided for "s + prompt + " specifier.\n"s);
            }
        }
        namesOfLoadedParameters.insert(parameter.name);
        argsIt++;
    }

    verifyRequirements();
}

int CommandLineParameters::getNumOfParametersRequiringValue() const noexcept
{
    const auto numOfObligatoryParameters{
        std::ranges::distance(std::views::values(parameters) | std::views::filter([](const auto &parameter) { return parameter.needsValue; }))};
    return static_cast<int>(numOfObligatoryParameters);
}

int CommandLineParameters::getMaxNumberOfEntries() const noexcept
{
    const auto maxNumOfEntries{parameters.size() + getNumOfParametersRequiringValue()};
    return static_cast<int>(maxNumOfEntries);
}

int CommandLineParameters::getMinNumberOfEntries() const noexcept
{
    return 1;
}

void CommandLineParameters::addParameter(const CommandLineParameter &parameter)
{
    parameters.insert({parameter.name, parameter});
    prompts.insert({parameter.longPrompt, parameter.shortPrompt});

    promptToNameMap.insert({parameter.longPrompt, parameter.name});
    promptToNameMap.insert({parameter.shortPrompt, parameter.name});
}

void CommandLineParameters::printHelp() const noexcept
{
    std::vector<std::pair<std::string, CommandLineParameter>> vectOfParams(parameters.begin(), parameters.end());
    std::ranges::sort(vectOfParams, [](auto &param1, auto &param2) { return param1.second.id < param2.second.id; });
    std::print("{} command line arguments:\n", appName);
    const auto nameWidht{getWidthOfTheLongestName()};
    const auto longNameWidht{getWidhtOfTheLongestLongPrompt()};
    for (const auto &param : std::views::elements<1>(vectOfParams))
    {
        std::print("{:>{}}, {:>3}, {}, {}\n", param.longPrompt, longNameWidht + 2, param.shortPrompt, param.name, param.description);
    }
}

bool CommandLineParameters::wasLoaded(const std::string &parameterName) const noexcept
{
    return namesOfLoadedParameters.contains(parameterName);
}

const CommandLineParameter &CommandLineParameters::getParameter(const std::string &parameterName) const
{
    return parameters.at(parameterName);
}

std::string CommandLineParameters::getValueOrEmptyString(const std::string &parameterName) const
{
    const auto &parameter{parameters.at(parameterName)};
    if (parameter.needsValue)
    {
        return parameter.value.value_or(""s);
    }
    else
    {
        return ""s;
    }
}

CommandLineParameter &CommandLineParameters::getArgumentByPrompt(const std::string &prompt)
{
    if (!promptToNameMap.contains(prompt))
    {
        throw std::runtime_error("The argument specifier: "s + prompt + " is not valid for this application.\n");
    }
    const auto name{promptToNameMap.at(prompt)};
    return parameters.at(name);
}

void CommandLineParameters::clear() noexcept
{
    namesOfLoadedParameters.clear();
}

std::string CommandLineParameters::getObligatoryStr(bool obligatory) const noexcept
{
    if (obligatory)
    {
        return "mandatory"s;
    }
    else
    {
        return "optional"s;
    }
}
int CommandLineParameters::getWidthOfTheLongestName() const noexcept
{
    auto paramWithLongestName{std::ranges::max_element(parameters | std::views::values, {}, [](const auto &t) { return t.name.length(); })};
    return (*paramWithLongestName).name.length();
}

int CommandLineParameters::getWidhtOfTheLongestLongPrompt() const noexcept
{
    auto paramWithLongestLongName{std::ranges::max_element(parameters | std::views::values, {}, [](const auto &t) { return t.longPrompt.length(); })};
    return (*paramWithLongestLongName).longPrompt.length();
}
bool CommandLineParameters::isPrompt(std::string_view str) const noexcept
{
    return prompts.contains(std::string{str});
}

int CommandLineParameters::countParametersThatAreMissingValue() const noexcept
{
    const auto isObligatory = [](const auto &p) { return p.isObligatory; };
    const auto hasNoValue = [](const auto &p) { return !p.value.has_value(); };
    const auto numOfInvalidParams =
        std::ranges::distance(parameters | std::views::values | std::views::filter(isObligatory) | std::views::filter(hasNoValue));
    return static_cast<int>(numOfInvalidParams);
}

bool CommandLineParameters::areMutualRequirementsFullfilled() const noexcept
{
    // const auto hasValue = [](const auto& p) { return (p.value.has_value() || !p.needsValue); };
    const auto needsOther = [](const auto &p) { return !p.needs.empty(); };
    const auto wasLoaded = [&](const auto &p) { return namesOfLoadedParameters.contains(p.name); };

    auto givenParametersThatNeedOthers = parameters | std::views::values | std::views::filter(wasLoaded) | std::views::filter(needsOther);

    bool requirementsAreFullfilled{true};
    for (const auto &parameterInNeed : std::move(givenParametersThatNeedOthers))
    {
        for (const auto &paramName : parameterInNeed.needs)
        {
            if (!namesOfLoadedParameters.contains(paramName))
            {
                std::print("Providing '{}' argument requires providing '{}' argument, but this one was not provided!\n", parameterInNeed.name,
                           paramName);
                requirementsAreFullfilled = false;
            }
        }
    }
    return requirementsAreFullfilled;
}

void CommandLineParameters::verifyRequirements() const
{
    const auto numOfParamtersThatMissValue = countParametersThatAreMissingValue();
    if (numOfParamtersThatMissValue > 0) [[unlikely]]
    {
        throw std::logic_error("There is/are "s + std::to_string(numOfParamtersThatMissValue) + " parameter(s), for which value was not provied. "s +
                               helpSuggestion);
    }
    else [[likely]]
    {
        const auto mutualRequirementsFullfilled{areMutualRequirementsFullfilled()};
        if (mutualRequirementsFullfilled) [[likely]]
        {
            return;
        }
        else
        {
            throw std::logic_error("Parameter logic error. Not all required command line arguments are provided! " + helpSuggestion);
        }
    }
}

} // namespace opall_solver_app