#include "opall/cost_function_data_creator.hpp"

#include "opall/evaluators.hpp"
#include "uncertainty_conversion.hpp"

namespace opall::cost_function_data
{
CostFunctionDataContainer createCostFunctionData(optimization_data_container::OptimizationDataContainer &optimizationDataContainer,
                                                 const ModellingConfig &config, [[maybe_unused]] ObservedPointIn3DStrategy &&strategy)
{
    CostFunctionDataContainer costFunctionDataContainer{};
    for (const auto &poseIdAndPointObservations : optimizationDataContainer.pointObservationContainer)
    {
        const auto &[poseId, pointObservations]{poseIdAndPointObservations};
        for (const auto &pointIdAndPointWithUncertainty : pointObservations)
        {
            const auto &[pointId, pointWithUncertainty]{pointIdAndPointWithUncertainty};
            const auto &[point, uncertainty]{pointWithUncertainty};

            const auto squareRootInformation{opall::computeSqareRootInformation(uncertainty)};

            // const auto poseIsFixed{optimizationDataContainer.fixedPosesSet.contains(poseId)};
            const auto poseIsFixed{optimizationDataContainer.posesContainer.at(poseId).type == Pose::Type::FIXED};
            if (poseIsFixed) [[unlikely]]
            {
                auto costFunctionData = ObservedPointIn3DFixedStationCostFunctionData(
                    poseId, pointId, optimizationDataContainer.pointContainer.at(pointId), optimizationDataContainer.posesContainer.at(poseId),
                    squareRootInformation, optimizationDataContainer.pointContainer.at(pointId).data(), config.lossFunctionOfObservedPoint);
                costFunctionDataContainer.insert(
                    CostFunctionData(std::move(costFunctionData), ObservedPointIn3DFixedStationCostFunctionDataEvaluator{}));
            }
            else [[likely]]
            {
                auto costFunctionData = ObservedPointIn3DCostFunctionData{poseId,
                                                                          pointId,
                                                                          point,
                                                                          squareRootInformation,
                                                                          optimizationDataContainer.pointContainer.at(pointId).data(),
                                                                          optimizationDataContainer.posesContainer.at(poseId).position.data(),
                                                                          optimizationDataContainer.posesContainer.at(poseId).quaternionWxyz.data(),
                                                                          config.lossFunctionOfObservedPoint};
                costFunctionDataContainer.insert(CostFunctionData(std::move(costFunctionData), ObservedPointIn3DCostFunctionDataEvaluator{}));
            }
        }
    }
    return costFunctionDataContainer;
}
} // namespace opall::cost_function_data