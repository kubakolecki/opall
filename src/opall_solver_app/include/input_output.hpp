#pragma once

#include "ReportConfig.hpp"
#include "ReportData.hpp"

#include <opall/data_types.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <filesystem>
#include <ostream>
#include <set>
#include <unordered_map>
#include <fstream>

using namespace std::literals;

namespace opall_solver_app
{
opall::PosesContainer readPoses(const std::filesystem::path pathToFile);
opall::PointObservationsContainer readPointObservations(const std::filesystem::path pathToFile);
void printPoints(const opall::PointContainer &points, std::ostream &outputStream, char separator, int precision);
std::string poseAsString(const opall::Pose &pose, char separator);
void printPoses(const opall::PosesContainer &poses, std::ostream &outputStream, char separator);
void printReport(const std::filesystem::path pathToFile, const ReportData &reportData, const ReportConfig &reportConfig);
void printResiduals(const opall::ResidualContainer& residualContainer, std::ostream &outputStream, char separator);


//void printMatrix(const std::filesystem::path pathToFile, const Eigen::SparseMatrix<double>& matrix, int precision);
//void printMatrix(const std::filesystem::path pathToFile, const Eigen::MatrixXd& matrix, int precision);

template <typename MatrixType>
void printMatrix(const std::filesystem::path pathToFile, const MatrixType& matrix, int precision)
{
    std::ofstream file{pathToFile};

    const auto rows{matrix.rows()};
    const auto cols{matrix.cols()};

    file << std::scientific << std::setprecision(precision);
    for (auto r{0}; r<rows; ++r)
    {
        for(auto c{0}; c<cols; ++c)
        {
            file << matrix.coeff(r,c);
            if (c == cols-1)
            {
                file <<'\n';
            }
            else
            {
                file <<',';
            }
        }
    }  
}


} // namespace opall_solver_app