#include "opall/full_covariance_computation.hpp"

#include <iostream>

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

    Eigen::MatrixXd computeUsingApproximateMatrixInversionEigen(const SparseJacobianData& jacobianData)
    {
        //Paper:
        //V. Y. Pan; L. Zhao; F. Soleymani
        //"An efficient computation of generalized inverse of a matrix"
        //Applied Mathematics and Computation, ISSN: 0096-3003, Vol: 316, Page: 89-101, 2018
 
        constexpr double mu {0.375};
        constexpr double psi {0.161794354839};
        constexpr double c1{0.944293637358057};
        constexpr double c2{-0.444293637358057};
        constexpr double c3{-0.0902857786190226};
        constexpr double d1{-0.284714221380977};
        constexpr double d2{-2.41091269024824};
        Eigen::SparseMatrix<double> jacobian{jacobianData.numberOfRows, jacobianData.numberOfColumns};

        jacobian.setFromTriplets(jacobianData.triplets.begin(), jacobianData.triplets.end());
        jacobian.makeCompressed();
        Eigen::SparseMatrix<double> hessian{jacobian.transpose() * jacobian};
        hessian.makeCompressed();


        std::cout << "jacobian\n";
        std::cout << jacobian <<'\n';

        std::cout << "hessian:\n";
        std::cout << hessian <<'\n';

        //Computing approximate inverse:
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> llt;
        llt.compute(hessian);

        if (llt.info() == Eigen::Success)
        {
            std::cout << "llt success" <<std::endl; 
        }
        else
        {
             std::cout << "llt FAILS!" <<std::endl;
             std::cout << "reason is " <<  llt.info() <<std::endl;
        }

        
        Eigen::SparseMatrix<double> L = llt.matrixL();
        L.makeCompressed();
        L.prune([=](const double& value, int /*row*/, int /*col*/) {
        return std::abs(value) >= 1e-04; });
        L.makeCompressed();

        for (int k = 0; k < L.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(L, k); it; ++it) {
                if (std::abs(it.value()) < 1e-4) {
                    std::cout << "Unexpected small value: " << it.value()
                            << " at (" << it.row() << "," << it.col() << ")\n";
                }
            }
        }




        std::cout << "L\n";
        std::cout << L << std::endl;

        Eigen::SparseMatrix<double> Linv = L.unaryExpr([](double x) {
            if (x<1e-04)
            {
                return x;
            }
            else
            {
                return 1.0 / x;
        }});
        
        std::cout <<"Linv\n";
        std::cout <<Linv << std::endl;
        

        Eigen::SparseMatrix<double> inverseApproximated = Linv.transpose() * Linv;

        std::cout << "inverse approximated\n";
        std::cout << inverseApproximated << "\n";

        std::cout << "testing approximates inverse\n";
        std::cout << inverseApproximated*hessian <<"\n";
        
        
        Eigen::VectorXd hessianDiagonalInv = hessian.diagonal().array().inverse();

        std::vector<Eigen::Triplet<double>> tripletListForInverse;
        const auto sizeOfMatrix{hessianDiagonalInv.size()};
        tripletListForInverse.reserve(sizeOfMatrix);
        for (auto i = 0; i < sizeOfMatrix; ++i) 
        {
            tripletListForInverse.emplace_back(i, i, hessianDiagonalInv(i));
        }
        Eigen::SparseMatrix<double> inverse{sizeOfMatrix, sizeOfMatrix};
        inverse.setFromTriplets(tripletListForInverse.begin(), tripletListForInverse.end());
        

        Eigen::SparseMatrix<double> I{sizeOfMatrix, sizeOfMatrix};
        I.setIdentity();
        for (auto iterationId {0}; iterationId < 3; ++iterationId )
        {
            
            std::cout << "***********************************************************************************************\n";
            std::cout << "iteration id = " << iterationId << "\n";
            std::cout << "inverse\n";
            std::cout << inverse << "\n";
            //inverse.prune(1e-05);
            //inverse.makeCompressed();
            Eigen::SparseMatrix<double> R = I - hessian*inverse;
            std::cout << "residual\n";
            std::cout << R << "\n";

            Eigen::SparseMatrix<double> R2 = R*R;
            Eigen::SparseMatrix<double> R4 = R2*R2;
            Eigen::SparseMatrix<double> M = (I + c1*R2 + R4)*(I + c2*R2 + R4);
            Eigen::SparseMatrix<double> T = M + c3*R2;
            Eigen::SparseMatrix<double> S = M + d1*R2 + d2*R4;
            Eigen::SparseMatrix<double> F = inverse * ((I+R)*((T*S) + mu*R2 + psi*R4));
            inverse =  F * inverse * F;

            std::cout << "R non-zeros: " << R.nonZeros() << std::endl;
            std::cout << "R2 non-zeros: " << R2.nonZeros() << std::endl;
            std::cout << "R4 non-zeros: " << R4.nonZeros() << std::endl;
        }

        return inverse;
    }

}