#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

using namespace std::string_literals;

namespace opall_solver_app
{

struct CommandLineParameter
{
    int id{0};
    std::string name;
    std::string shortPrompt;
    std::string longPrompt;
    bool isObligatory{false};
    bool needsValue{false};
    std::optional<std::string> value;
    std::string description;
    std::unordered_set<std::string> needs{};
};

class CommandLineParameters
{
  public:
    CommandLineParameters(const std::string_view &appName);
    void parseParameters(int argc, char **argv);
    int getNumOfParametersRequiringValue() const noexcept;
    int getMaxNumberOfEntries() const noexcept;
    int getMinNumberOfEntries() const noexcept;
    void addParameter(const CommandLineParameter &parameter);
    void printHelp() const noexcept;
    bool wasLoaded(const std::string &parameterName) const noexcept;
    const CommandLineParameter &getParameter(const std::string &parameterName) const;
    std::string getValueOrEmptyString(const std::string &parameterName) const;
    void clear() noexcept;

  private:
    std::string getObligatoryStr(bool obligatory) const noexcept;
    int getWidthOfTheLongestName() const noexcept;
    int getWidhtOfTheLongestLongPrompt() const noexcept;
    bool isPrompt(std::string_view str) const noexcept;
    int countParametersThatAreMissingValue() const noexcept;
    bool areMutualRequirementsFullfilled() const noexcept;
    void verifyRequirements() const;
    CommandLineParameter &getArgumentByPrompt(const std::string &prompt);

    std::string_view appName{};
    std::unordered_set<std::string> prompts;
    std::unordered_map<std::string, std::string> promptToNameMap;
    std::unordered_set<std::string> namesOfLoadedParameters;
    std::unordered_map<std::string, CommandLineParameter> parameters;
    const std::string helpSuggestion{"Call program with - h or --help argument to display information about program arguments.\n"s};
};

} // namespace opall_solver_app