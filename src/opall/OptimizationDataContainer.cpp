#include "opall/OptimizationDataContainer.hpp"

#include <ranges>

namespace opall::optimization_data_container
{

ConsistencyEvaluationResult checkConsistency(const OptimizationDataContainer &optimizationDataContainer)
{
    const auto idOfPosesFromPoses{optimizationDataContainer.posesContainer | std::ranges::views::keys | std::ranges::to<opall::PosesIds>()};
    const auto idOfPosesFromPosesFromPointObservations{optimizationDataContainer.pointObservationContainer | std::ranges::views::keys |
                                                       std::ranges::to<opall::PosesIds>()};

    if (idOfPosesFromPoses != idOfPosesFromPosesFromPointObservations)
    {
        return ConsistencyEvaluationResult::POSE_IDS_INCONSISTENT;
    }

    return ConsistencyEvaluationResult::SUCCESS;
}

} // namespace opall::optimization_data_container
