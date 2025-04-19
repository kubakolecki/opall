#pragma once

#include "ReportConfig.hpp"
#include "ReportData.hpp"

#include <opall/data_types.hpp>

#include <Eigen/Dense>

#include <filesystem>
#include <ostream>
#include <set>
#include <unordered_map>

using namespace std::literals;

namespace opall_solver_app
{
opall::PosesContainer readPoses(const std::filesystem::path pathToFile);
opall::PointObservationsContainer readPointObservations(const std::filesystem::path pathToFile);
void printPoints(const opall::PointContainer &points, std::ostream &outputStream, char separator, int precision);
std::string poseAsString(const opall::Pose &pose, char separator);
void printPoses(const opall::PosesContainer &poses, std::ostream &outputStream, char separator);
void printReport(const std::filesystem::path pathToFile, const ReportData &reportData, const ReportConfig &reportConfig);

} // namespace opall_solver_app