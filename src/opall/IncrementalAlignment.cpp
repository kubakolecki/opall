#include "opall/IncrementalAlignment.hpp"
#include "opall/Problem.hpp"
#include "opall/cost_function_data_creator.hpp"
#include "opall/fill_problem.hpp"
#include "opall/optimization_data_processor.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <execution>
#include <print> //TODO: remove when not needed
#include <ranges>
#include <set>
#include <vector>

namespace opall
{
IncrementalAlignment::IncrementalAlignment(const ModellingConfig &modellingConfig, const OptimizationConfig &optimizationConfig)
    : m_modellingConfig(modellingConfig), m_optimizationConfig(optimizationConfig)
{
}

std::pair<optimization_data_container::OptimizationDataContainer, IncrementalAlignmentStatus> IncrementalAlignment::align(
    const optimization_data_container::OptimizationDataContainer &dataContainer) const
{
    std::print("Running incremental alignment.\n");
    optimization_data_container::OptimizationDataContainer alignedDataContainer;

    // TODO: don't need variable idsOfPoses? Refactor next 2 lines
    // const auto idsOfPoses{dataContainer.pointObservationContainer | std::ranges::views::keys | std::ranges::to<std::vector<PoseId>>()};
    auto idsOfPosesRemainingToAlign{dataContainer.pointObservationContainer | std::ranges::views::keys | std::ranges::to<std::list<PoseId>>()};

    alignedDataContainer.posesContainer.insert({idsOfPosesRemainingToAlign.front(), Pose{Position{}, {1.0, 0.0, 0.0, 0.0}, Pose::Type::FIXED}});
    alignedDataContainer.pointObservationContainer.insert(
        {idsOfPosesRemainingToAlign.front(), dataContainer.pointObservationContainer.at(idsOfPosesRemainingToAlign.front())});
    // alignedDataContainer.fixedPosesSet.insert(idsOfPosesRemainingToAlign.front());
    idsOfPosesRemainingToAlign.erase(idsOfPosesRemainingToAlign.begin());
    // TODO: handle the case if there are only 2 poses

    alignedDataContainer.pointContainer =
        computePointCoordinates(alignedDataContainer.pointObservationContainer, alignedDataContainer.posesContainer);

    while (!idsOfPosesRemainingToAlign.empty())
    {
        const auto idsOfTargetPoints{alignedDataContainer.pointContainer | std::ranges::views::keys | std::ranges::to<std::set<PointId>>()};
        const auto itIdWithMostCorrespondences{
            findPoseWithMostCorrespondences(idsOfPosesRemainingToAlign, dataContainer.pointObservationContainer, idsOfTargetPoints)};
        const auto idOfPoseSelectedToAlign{*itIdWithMostCorrespondences};
        const auto idsCorrespondences{computeIdsOfCorrespondences(dataContainer.pointObservationContainer.at(idOfPoseSelectedToAlign) |
                                                                      std::ranges::views::keys | std::ranges::to<std::set<PointId>>(),
                                                                  idsOfTargetPoints)};
        const auto numberOfCorrespondences{idsCorrespondences.size()};
        const auto &pointObservationsForPoseBeingAligned{dataContainer.pointObservationContainer.at(idOfPoseSelectedToAlign)};

        std::print("Trying to align pose {}...", idOfPoseSelectedToAlign);

        if (numberOfCorrespondences < 3)
        {
            std::print("Incremental alignemnt failed for pose {}.\n", idOfPoseSelectedToAlign);
            return std::make_pair(std::move(alignedDataContainer), IncrementalAlignmentStatus::TO_FEW_CORRESPONDENCES);
        }

        const auto &[sourcePoints, targetPoints]{
            createEigenUmayamaInput(idsCorrespondences, pointObservationsForPoseBeingAligned, alignedDataContainer.pointContainer)};

        const Eigen::Matrix4d isometry{Eigen::umeyama(sourcePoints, targetPoints, false)};

        alignedDataContainer.posesContainer.insert({idOfPoseSelectedToAlign, eigenIsometryToPose(isometry)});
        alignedDataContainer.pointObservationContainer.insert(
            {idOfPoseSelectedToAlign, dataContainer.pointObservationContainer.at(idOfPoseSelectedToAlign)});

        alignedDataContainer.pointContainer =
            computePointCoordinates(alignedDataContainer.pointObservationContainer, alignedDataContainer.posesContainer);

        opall::Problem optimizationProblem;
        const auto costFunctionDataContainer{opall::cost_function_data::createCostFunctionData(
            alignedDataContainer, m_modellingConfig, opall::cost_function_data::ObservedPointIn3DStrategy{})};

        opall::fillProblem(costFunctionDataContainer, optimizationProblem);
        const auto solverSummary{optimizationProblem.solve(m_optimizationConfig)};

        if (!solverSummary.isSolutionUsable)
        {
            std::print("Incremental alignemnt failed for pose {}.\n", idOfPoseSelectedToAlign);
            return std::make_pair(std::move(alignedDataContainer), IncrementalAlignmentStatus::LEAST_SQUARE_SOLVER_FAILED);
        }

        idsOfPosesRemainingToAlign.erase(itIdWithMostCorrespondences);
    }
    return std::make_pair(std::move(alignedDataContainer), IncrementalAlignmentStatus::SUCESS);
}

std::vector<PointId> IncrementalAlignment::computeIdsOfCorrespondences(const std::set<PointId> &ids1, const std::set<PointId> &ids2) const
{
    std::vector<PointId> idsOfCorrespondences;
    idsOfCorrespondences.reserve(ids1.size());
    std::ranges::set_intersection(ids1, ids2, std::back_inserter(idsOfCorrespondences));
    idsOfCorrespondences.shrink_to_fit();
    return idsOfCorrespondences;
}

Pose IncrementalAlignment::eigenIsometryToPose(const Eigen::Matrix4d &isometry) const
{
    const auto eigenQuaternion{Eigen::Quaterniond{isometry.block<3, 3>(0, 0)}};
    QuaternionWxyz quaternion{eigenQuaternion.w(), eigenQuaternion.x(), eigenQuaternion.y(), eigenQuaternion.z()};
    return Pose{isometry.block<3, 1>(0, 3), std::move(quaternion), Pose::Type::FREE};
}

std::list<PoseId>::const_iterator IncrementalAlignment::findPoseWithMostCorrespondences(const std::list<PoseId> &idsOfPosesToCheck,
                                                                                        const PointObservationsContainer &pointObservations,
                                                                                        const std::set<PointId> &referencePointIds) const
{
    // TODO: consider using parallel policy
    auto poseIdIterator = std::ranges::max_element(idsOfPosesToCheck, [&](const auto poseId1, const auto poseId2) {
        const auto idsOfMeasuredPoints1 = pointObservations.at(poseId1) | std::ranges::views::keys | std::ranges::to<std::set<PointId>>();
        const auto idsOfMeasuredPoints2 = pointObservations.at(poseId2) | std::ranges::views::keys | std::ranges::to<std::set<PointId>>();
        const auto idsCorrespondences1 = computeIdsOfCorrespondences(idsOfMeasuredPoints1, referencePointIds);
        const auto idsCorrespondences2 = computeIdsOfCorrespondences(idsOfMeasuredPoints2, referencePointIds);
        return idsCorrespondences1 < idsCorrespondences2;
    });

    return poseIdIterator;
}

std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> IncrementalAlignment::createEigenUmayamaInput(const std::vector<PointId> &idsOfCorrespondences,
                                                                                            const PointObservations &sourceRange,
                                                                                            const PointContainer &targetRange) const
{
    Eigen::Matrix3Xd sourcePoints{3, idsOfCorrespondences.size()};
    Eigen::Matrix3Xd targetPoints{3, idsOfCorrespondences.size()};

    const auto enumeratedView{std::views::enumerate(idsOfCorrespondences)};

    std::for_each(std::execution::par, enumeratedView.begin(), enumeratedView.end(), [&](const auto &matrixColumnIndexAndpointId) {
        const auto &[matrixColumnIndex, pointId]{matrixColumnIndexAndpointId};
        sourcePoints.block<3, 1>(0, matrixColumnIndex) = sourceRange.at(pointId).first;
        targetPoints.block<3, 1>(0, matrixColumnIndex) = targetRange.at(pointId);
    });

    return {std::move(sourcePoints), std::move(targetPoints)};
}

} // namespace opall