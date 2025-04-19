#pragma once

namespace opall
{

struct OptimizationConfig
{
    enum class SparseAlgebraEngine
    {
        SUIT_SPARSE,
        EIGEN_SPARSE
    };
    enum class LinearSolverType
    {
        DENSE_QR,
        SPARSE_NORMAL_CHOLESKY,
        DENSE_NORMAL_CHOLESKY,
        DENSE_SCHUR,
        SPARSE_SCHUR,
        ITERATIVE_SCHUR,
        CGNR
    };

    SparseAlgebraEngine sparseAlgebraEngine{SparseAlgebraEngine::SUIT_SPARSE};
    LinearSolverType linearSolverType{LinearSolverType::SPARSE_NORMAL_CHOLESKY};
    bool printOptimizationSteps{true};
    bool printSolverReport{true};

    int maxNumberOfIterations{30};
    int numOfThreads{1};
    double functionTolerance{1e-6};
    double gradientTolerance{1e-10};
};

} // namespace opall