#pragma once
#include "opall/data_types.hpp"

#include <Eigen/Core>
#include <ceres/rotation.h>

namespace opall::cost_functors
{

struct ObservedPointIn3DFixedStationCostFunctor
{
    // w - world c.s.
    // m - mapping c.s. (e.g. point cloud c.s.)
    // p - point in 3D

    explicit ObservedPointIn3DFixedStationCostFunctor(const Eigen::Vector3d &observedPoint, const Eigen::Vector3d &posePosition,
                                                      const Eigen::Vector4d &poseWxyzQuaternion,
                                                      const uncertainty3d::SquareRootInformationMatrix &squareRootInformationMatrix)
        : observedPoint{observedPoint}, posePosition{posePosition}, poseWxyzQuaternion{poseWxyzQuaternion},
          squareRootInformationMatrix{squareRootInformationMatrix}
    {
    }

    template <typename T>
    bool operator()(const T *const point_wp_w, T *residual) const
    {
        const T point_mp_w[3]{
            point_wp_w[0] - posePosition[0],
            point_wp_w[1] - posePosition[1],
            point_wp_w[2] - posePosition[2],
        };

        const T poseWxyzQuaternion_w_m[4]{
            // TODO: this can be computed diretly in constructor?
            T{poseWxyzQuaternion[0]},
            T{-poseWxyzQuaternion[1]},
            T{-poseWxyzQuaternion[2]},
            T{-poseWxyzQuaternion[3]},
        };

        T point_mp_m[3]{T{0.0}, T{0.0}, T{0.0}};

        ceres::UnitQuaternionRotatePoint(poseWxyzQuaternion_w_m, point_mp_w, point_mp_m);

        T residualBeforeNormalization[3]{point_mp_m[0] - T(observedPoint(0)), point_mp_m[1] - T(observedPoint(1)),
                                         point_mp_m[2] - T(observedPoint(2))};

        Eigen::Map<Eigen::Matrix<T, 3, 1>> residualBeforeNormalizationEigen(residualBeforeNormalization);
        Eigen::Matrix<T, 3, 1> normalizedResiduals{squareRootInformationMatrix.matrix.template cast<T>() * residualBeforeNormalizationEigen};

        residual[0] = normalizedResiduals(0);
        residual[1] = normalizedResiduals(1);
        residual[2] = normalizedResiduals(2);

        return true;
    }

    Eigen::Vector3d observedPoint;
    Eigen::Vector3d posePosition;
    Eigen::Vector4d poseWxyzQuaternion;
    uncertainty3d::SquareRootInformationMatrix squareRootInformationMatrix;
};

} // namespace opall::cost_functors