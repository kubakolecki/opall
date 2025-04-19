#include "opall/evaluators.hpp"
#include "cost_functors/ObservedPointIn3DCostFunctor.hpp"
#include "cost_functors/ObservedPointIn3DFixedStationCostFunctor.hpp"
#include "opall/geometry.hpp"

#include <Eigen/Dense>

namespace opall
{

EvaluationResult ObservedPointIn3DFixedStationCostFunctionDataEvaluator::operator()(
    const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData) const
{
    const auto costFunctor{cost_functors::ObservedPointIn3DFixedStationCostFunctor{
        costFunctionData.observedPoint, costFunctionData.pose.position, costFunctionData.pose.quaternionWxyz, costFunctionData.uncertainty}};
    Eigen::Vector3d residuals;
    costFunctor(costFunctionData.point, residuals.data());
    Eigen::Vector3d deNormlizedResiudals{costFunctionData.uncertainty.matrix.ldlt().solve(Eigen::Matrix3d::Identity()) * residuals};
    return EvaluationResult(deNormlizedResiudals);
}

EvaluationResult ObservedPointIn3DCostFunctionDataEvaluator::operator()(
    const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData) const
{
    const auto costFuntor{cost_functors::ObservedPointIn3DCostFunctor{costFunctionData.observedPoint, costFunctionData.uncertainty}};
    Eigen::Vector3d residuals;
    costFuntor(costFunctionData.posePosition, costFunctionData.poseWxyzQuaternion, costFunctionData.point, residuals.data());
    Eigen::Vector3d deNormlizedResiudals{costFunctionData.uncertainty.matrix.ldlt().solve(Eigen::Matrix3d::Identity()) * residuals};
    return EvaluationResult(deNormlizedResiudals);
}

} // namespace opall
