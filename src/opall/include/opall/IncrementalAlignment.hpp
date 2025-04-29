#pragma once

#include "opall/ModellingConfig.hpp"
#include "opall/OptimizationConfig.hpp"
#include "opall/OptimizationDataContainer.hpp"

#include <Eigen/Core>

#include <set>
#include <list>

// #include <vector>

namespace opall
{

enum class IncrementalAlignmentStatus
{
    SUCESS,
    TO_FEW_CORRESPONDENCES,
    LEAST_SQUARE_SOLVER_FAILED
};

class IncrementalAlignment
{
  public:
    IncrementalAlignment(const ModellingConfig &modellingConfig, const OptimizationConfig &optimizationConfig);
    std::pair<optimization_data_container::OptimizationDataContainer, IncrementalAlignmentStatus> align(
        const optimization_data_container::OptimizationDataContainer &dataContainer) const;

  private:
    ModellingConfig m_modellingConfig;
    OptimizationConfig m_optimizationConfig;

    std::vector<PointId> computeIdsOfCorrespondences(const std::set<PointId> &ids1, const std::set<PointId> &ids2) const;
    Pose eigenIsometryToPose(const Eigen::Matrix4d &isometry) const;
    std::list<PoseId>::const_iterator findPoseWithMostCorrespondences(const std::list<PoseId> &idsOfPosesToCheck,
                                                                      const PointObservationsContainer &pointObservations,
                                                                      const std::set<PointId> &referencePointIds) const;

    std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xd> createEigenUmayamaInput(const std::vector<PointId> &idsOfCorrespondences,
                                                                          const PointObservations &sourceRange,
                                                                          const PointContainer &targetRange) const;
};

} // namespace opall
