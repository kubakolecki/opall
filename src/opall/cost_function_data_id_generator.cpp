#include "opall/cost_function_data_id_generator.hpp"

using namespace std::literals::string_literals;

namespace opall::cost_function_id_generator
{

CostFunctionIdentifier generate(const cost_function_data::ObservedPointIn3DCostFunctionData &costFunctionData)
{
    std::string pointId{std::to_string(costFunctionData.pointId)};
    std::string poseId{std::to_string(costFunctionData.poseId)};
    const CostFunctionIdentifier identifier{"observed_point_in_3D"s, poseId, pointId};
    return identifier;
}

CostFunctionIdentifier generate(const cost_function_data::ObservedPointIn3DFixedStationCostFunctionData &costFunctionData)
{
    std::string pointId{std::to_string(costFunctionData.pointId)};
    std::string poseId{std::to_string(costFunctionData.poseId)};
    const CostFunctionIdentifier identifier{"observed_point_in_3D_from_fixed_station"s, poseId, pointId};
    return identifier;
}

} // namespace opall::cost_function_id_generator
