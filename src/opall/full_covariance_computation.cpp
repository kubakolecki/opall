#include "opall/full_covariance_computation.hpp"

namespace opall::covariance
{
    Eigen::MatrixXd computeFullCovariance(const SparseJacobianData& jacobianData, FullCovarianceComputer fullCovarianceComputer)
    {
        Eigen::MatrixXd covarianceMatrix{fullCovarianceComputer(jacobianData)};
        return covarianceMatrix;
    }

    Eigen::MatrixXd computeUsingNaiveMatrixInversion(const SparseJacobianData& jacobianData)
    {
        Eigen::SparseMatrix<double> jacobian(jacobianData.numberOfRows, jacobianData.numberOfColumns);
        jacobian.setFromTriplets(jacobianData.triplets.begin(), jacobianData.triplets.end());
        jacobian.makeCompressed();
        Eigen::SparseMatrix<double> hessian{jacobian.transpose() * jacobian};
        hessian.makeCompressed();
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > solver;
        solver.compute(hessian);

        Eigen::SparseMatrix<double> identity(hessian.cols(), hessian.rows());
        identity.setIdentity();
        Eigen::MatrixXd covariance{solver.solve(identity)};
        return covariance;
    }

}