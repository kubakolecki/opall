#pragma once

#include <string>

namespace opall
{
struct SolverSummary
{
    std::string fullReport{};
    bool isSolutionUsable{false};
    double initialCost{0.0};
    double finalCost{0.0};
    double totalTimeInSeconds{0.0};
    int numOfParameterBlocks{0};
    int numOfParameters{0};
    int numOfEffectiveParameters{0};
    int numOfResidualBlocks{0};
    int numOfResiduals{0};
    int numOfThreadsUsed{1};
    double sigmaZero{1.0};
};

} // namespace opall