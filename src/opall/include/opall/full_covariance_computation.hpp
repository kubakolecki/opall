#pragma once
#include "opall/SparseJacobianData.hpp"

#include <functional>

namespace opall::covariance
{
    using FullCovarianceComputer = std::function<Eigen::MatrixXd(const SparseJacobianData&)>;
    Eigen::MatrixXd computeFullCovariance(const SparseJacobianData& jacobianData, FullCovarianceComputer fullCovarianceComputer);

    Eigen::MatrixXd computeUsingNaiveMatrixInversion(const SparseJacobianData& jacobianData);

}