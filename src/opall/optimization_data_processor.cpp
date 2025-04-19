#include "opall/optimization_data_processor.hpp"
#include "opall/geometry.hpp"

#include <algorithm>
#include <exception>
#include <ranges>
#include <source_location>

namespace opall
{

PointContainer computePointCoordinates(const PointObservationsContainer &pointObservationsContainer, const PosesContainer &posesContainer)
{
    auto pointContainer{PointContainer{}};

    for (const auto &poseIdAndPointObservations : pointObservationsContainer)
    {
        const auto &[poseId, pointObservations]{poseIdAndPointObservations};
        if (!posesContainer.contains(poseId))
        {
            const auto cppFilename = std::string{std::source_location::current().file_name()};
            throw std::runtime_error("Error in : " + cppFilename + " No corresponding pose for point observations");
        }

        auto pose{posesContainer.at(poseId)};

        auto transformation = [&pose](const auto &pointWithUncertainty) {
            const auto &[pointId, positionAndUncertainty]{pointWithUncertainty};
            const auto &[point, uncertainty]{positionAndUncertainty};
            const auto transformedPoint{transformPointUsingPose(point, pose)};
            return std::make_pair(pointId, transformedPoint);
        };

        auto requiresComputation = [&pointContainer](const auto &pointWithUncertainty) {
            return !pointContainer.contains(pointWithUncertainty.first);
        };
        std::ranges::transform(pointObservations | std::views::filter(requiresComputation), std::inserter(pointContainer, pointContainer.begin()),
                               transformation);
    }
    return pointContainer;
}

} // namespace opall