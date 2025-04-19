#pragma once

#include "data_types.hpp"

namespace opall::optimization_data_container
{

struct OptimizationDataContainer
{
    PointObservationsContainer pointObservationContainer;
    PosesContainer posesContainer;
    PointContainer pointContainer;
};

enum class ConsistencyEvaluationResult
{
    SUCCESS,
    POSE_IDS_INCONSISTENT
};

ConsistencyEvaluationResult checkConsistency(const OptimizationDataContainer &optimizationDataContainer);

} // namespace opall::optimization_data_container