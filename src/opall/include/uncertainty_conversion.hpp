#pragma once

#include <Eigen/Dense>
#include <variant>
#include <iostream> //TODO: remove when not needed

#include "opall/data_types.hpp"

namespace opall
{
template <class... Ts>
struct OverloadedUncertaintyConverter3d : Ts...
{
    using Ts::operator()...;
};

uncertainty3d::SquareRootInformationMatrix computeSqareRootInformation(const uncertainty3d::Uncertainty &uncertainty)
{
    uncertainty3d::SquareRootInformationMatrix squareRootInformation =
        std::visit(OverloadedUncertaintyConverter3d{
                       [](const uncertainty3d::StandardDeviation &standardDeviation) {
                           uncertainty3d::SquareRootInformationMatrix squareRootInformationMatrix{Eigen::Matrix3d::Zero()};
                           squareRootInformationMatrix.matrix(0, 0) = 1 / standardDeviation(0);
                           squareRootInformationMatrix.matrix(1, 1) = 1 / standardDeviation(1);
                           squareRootInformationMatrix.matrix(2, 2) = 1 / standardDeviation(2);
                           return squareRootInformationMatrix;
                       },
                       [](const uncertainty3d::CovarianceMatrix &covarianceMatrix) {
                           std::cout <<"computing square root information matrix from covariance matrix...\n";
                           Eigen::LLT<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> choleskyDecomposition(covarianceMatrix.matrix);
                           Eigen::Matrix<double, 3, 3, Eigen::RowMajor> informationMatrix{choleskyDecomposition.matrixL()};
                           Eigen::Matrix<double, 3, 3, Eigen::RowMajor> squareRootInformationMatrix{informationMatrix.inverse()};
                           return uncertainty3d::SquareRootInformationMatrix{squareRootInformationMatrix};
                       },
                       [](const uncertainty3d::InformationMatrix &informationMatrix) {
                           Eigen::LLT<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> choleskyDecomposition(informationMatrix.matrix);
                           Eigen::Matrix<double, 3, 3, Eigen::RowMajor> squareRootInformationMatrix{choleskyDecomposition.matrixL()};
                           return uncertainty3d::SquareRootInformationMatrix{squareRootInformationMatrix};
                       },
                       [](const uncertainty3d::SquareRootInformationMatrix &squareRootInformationMatrix) { return squareRootInformationMatrix; }},
                   uncertainty);
    return squareRootInformation;
}

} // namespace opall