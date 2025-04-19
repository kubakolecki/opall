#include "input_output.hpp"

#include <charconv>
#include <chrono>
#include <exception>
#include <fstream>
#include <optional>
#include <print>
#include <ranges>
#include <string>
#include <system_error>

namespace opall_solver_app
{
double checkQuaternionNorm(const opall::QuaternionWxyz &quaternion)
{
    const auto norm{quaternion.norm()};

    if (quaternion.norm() - 1.0 > 1e-6)
    {
        std::println("As an input pose data a unit quaternion is required to represent rotation. The following quaternion:");
        std::println("{:.10f}, {:.10f}, {:.10f}, {:.10f}", quaternion(0), quaternion(1), quaternion(2), quaternion(3));
        std::println("is not of unit norm! Check if data in the file are not truncated.");
        throw std::invalid_argument("Wrong input data! Norm of quaternion differs significantly from 1: norm = " + std::to_string(norm));
    }

    return norm;
}

using PoseWithId = std::pair<opall::PoseId, opall::Pose>;
using Point3dWithId = std::pair<opall::PointId, opall::point_3d::PointWithUncertainty>;

// template <typename N>
// struct NumberOfPointDataEntries
//{
// };

PoseWithId tryParsePoseData(const std::vector<std::string> &poseEntries)
{
    if (poseEntries.size() != 9)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for pose! Data record is missformated!");
    }

    const auto poseTypeString{poseEntries.back()};
    if (!opall::poseTypeStrToPoseType.contains(poseTypeString))
    {
        throw std::invalid_argument("Fatal error. Unerecognized pose type: " + poseTypeString);
    }

    opall::Pose pose;
    pose.type = opall::poseTypeStrToPoseType.at(poseTypeString);

    for (const auto [index, entry] : poseEntries | std::views::enumerate | std::views::drop(1) | std::views::take(7))
    {
        const auto poseEntryIdx{index - 1};
        auto value{0.0};
        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec == std::errc()) [[likely]]
        {
            if (std::isnan(value) || std::isinf(value))
            {
                throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in pose data");
            }

            if (poseEntryIdx < 3)
            {
                pose.position(poseEntryIdx) = value;
            }
            else
            {
                if (std::abs(value) > 1.0) [[unlikely]]
                {
                    throw std::invalid_argument("Unit quaternion entry is not in <-1, +1> interval!");
                }
                else [[likely]]
                {
                    pose.quaternionWxyz(poseEntryIdx - 3) = value;
                }
            }
        }
        else [[unlikely]]
        {
            throw std::invalid_argument("Not a numeric entry found for position or quaternion, or data record is missformated!");
        }
    }

    opall::PoseId poseId;

    if (std::from_chars(poseEntries[0].c_str(), poseEntries[0].c_str() + poseEntries[0].length(), poseId).ec != std::errc())
    {
        throw std::invalid_argument("Invalid pose ID: "s + poseEntries[0]);
    }

    const auto norm{checkQuaternionNorm(pose.quaternionWxyz)};
    pose.quaternionWxyz *= (1.0 / norm);
    return std::make_pair(poseId, pose);
}

template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};

Point3dWithId tryParsePointObservationData(const std::vector<std::string> &pointEntries)
{
    const auto numberOfEntries{pointEntries.size()};

    if (numberOfEntries != 8 && numberOfEntries != 14)
    {
        throw std::invalid_argument("Fatal error. Invalid number of entries for point observations! Data record is missformated!");
    }

    opall::point_3d::PointWithUncertainty point;
    auto &[pointCoordinates, pointUncertainty]{point};
    if (numberOfEntries == 8)
    {
        pointUncertainty = opall::uncertainty3d::StandardDeviation{};
    }
    if (numberOfEntries == 14)
    {
        pointUncertainty = opall::uncertainty3d::CovarianceMatrix{};
    }
    // TODO: consider cases where Information Matrix or Square Root Information Matrix is provided

    auto pointId{opall::PointId{}};

    if (std::from_chars(pointEntries[1].c_str(), pointEntries[1].c_str() + pointEntries[1].length(), pointId).ec != std::errc())
    {
        throw std::invalid_argument("Error. Point observation id must be an integer, instead found: " + pointEntries[1]);
    }

    for (const auto [index, entry] : pointEntries | std::views::enumerate | std::views::drop(2))
    {
        const auto pointEntryIdx{index - 2};
        auto value{0.0};
        if (std::from_chars(entry.c_str(), entry.c_str() + entry.length(), value).ec != std::errc())
        {
            throw std::invalid_argument("Not a numeric entry found for position or quaternion, or data record is missformated!");
        }

        if (std::isnan(value) || std::isinf(value)) [[unlikely]]
        {
            throw std::invalid_argument("Not a number (nan) or inifinite (inf) value detected in point observation data");
        }

        if (pointEntryIdx < 3)
        {
            pointCoordinates[pointEntryIdx] = value;
        }
        else
        {
            if (value <= 0.0) [[unlikely]]
            {
                throw std::invalid_argument("Error. Point observation uncertainty must be > 0, while found: " + std::to_string(value));
            }
            else [[likely]]
            {
                std::visit(overloaded{[idx = pointEntryIdx, val = value](opall::uncertainty3d::StandardDeviation &uncertainty) {
                                          uncertainty.data()[idx - 3] = val;
                                      },
                                      [idx = pointEntryIdx, val = value](opall::uncertainty3d::CovarianceMatrix &uncertainty) {
                                          uncertainty.matrix.data()[idx - 3] = val;
                                      },
                                      [idx = pointEntryIdx, val = value](opall::uncertainty3d::InformationMatrix &uncertainty) { /*TODO*/ },
                                      [idx = pointEntryIdx,
                                       val = value](opall::uncertainty3d::SquareRootInformationMatrix &uncertainty) { /*TODO*/ }},
                           pointUncertainty);
            }
        }
    }

    return std::make_pair(pointId, point);
}

} // namespace opall_solver_app

opall::PosesContainer opall_solver_app::readPoses(const std::filesystem::path pathToFile)
{
    if (!std::filesystem::exists(pathToFile))
    {
        throw std::invalid_argument("Fatal Error. File with poses does not exist!");
    }

    auto poses{opall::PosesContainer{}};

    auto file{std::ifstream{pathToFile}};
    std::string header;
    std::getline(file, header, '\n');

    for (std::string line; std::getline(file, line, '\n');)
    {
        std::istringstream dataRecord;
        dataRecord.str(line);
        std::vector<std::string> poseEntries;
        poseEntries.reserve(9);
        for (std::string poseEntry; std::getline(dataRecord, poseEntry, ',');)
        {
            poseEntries.push_back(poseEntry);
        }
        const auto poseWithId{tryParsePoseData(poseEntries)};
        const auto &[id, pose]{poseWithId};
        poses.insert({id, pose});
    }

    const auto fixedPoseIds{poses |
                            std::views::filter([](const auto &poseWithId) { return poseWithId.second.type == opall::Pose::Type::FIXED; }) |
                            std::ranges::views::keys | std::ranges::to<opall::PosesIds>()};

    if (poses.empty())
    {
        throw(std::logic_error("Error! Looks like file with poses is empty!"));
    }

    if (fixedPoseIds.size() > 1)
    {
        throw(std::logic_error("Error! In current implementation we allow for only 1 fixed pose!"));
    }

    return poses;
}

// opall::PointObservationsContainer createPointObservationContainer(const opall::PosesIds &posesIds)
//{
//     auto pointObservationContainer{opall::PointObservationsContainer{}};
//
//     for (const auto poseId : posesIds)
//     {
//         pointObservationContainer.insert({poseId, opall::PointObservations{}});
//     }
//
//     return pointObservationContainer;
// }

opall::PointObservationsContainer opall_solver_app::readPointObservations(const std::filesystem::path pathToFile)
{
    // auto pointObservationContainer{createPointObservationContainer(posesIds)};

    opall::PointObservationsContainer pointObservationContainer;

    if (!std::filesystem::exists(pathToFile)) [[unlikely]]
    {
        throw std::invalid_argument("Fatal Error. File with point observations does not exist!");
    }

    auto file{std::ifstream{pathToFile}};
    std::string header;
    std::getline(file, header, '\n');
    std::string line;

    for (std::string line; std::getline(file, line, '\n');)
    {
        std::istringstream dataRecord;
        dataRecord.str(line);
        std::vector<std::string> observationEntries;
        observationEntries.reserve(14);
        for (std::string observationEntry; std::getline(dataRecord, observationEntry, ',');)
        {
            observationEntries.push_back(observationEntry);
        }
        observationEntries.shrink_to_fit();

        opall::PoseId poseId;
        if (std::from_chars(observationEntries[0].c_str(), observationEntries[0].c_str() + observationEntries[0].length(), poseId).ec != std::errc())
        {
            throw std::invalid_argument("Point observation file contains invalid pose ID: "s + observationEntries[0]);
        }

        // const auto poseId{observationEntries[0]};
        const auto pointDataWithId{tryParsePointObservationData(observationEntries)};
        if (!pointObservationContainer.contains(poseId)) //[[unlikely]]
        {
            pointObservationContainer.insert({poseId, opall::PointObservations{}});
            // throw std::logic_error("Error while reading point observations. Pose with id = " + observationEntries[0] + " does not exist.");
        }
        pointObservationContainer.at(poseId).insert(pointDataWithId);
    }
    return pointObservationContainer;
}

void opall_solver_app::printPoints(const opall::PointContainer &points, std::ostream &outputStream, char separator, int precision)
{
    outputStream << std::fixed << std::setprecision(precision);
    auto printer = [&outputStream, separator = separator](const auto &idAndPoint) {
        const auto &[id, point]{idAndPoint};
        outputStream << id << separator;
        outputStream << point(0) << separator << point(1) << separator << point(2) << "\n";
    };

    std::ranges::for_each(points, printer);

    outputStream.flush();
}

std::string opall_solver_app::poseAsString(const opall::Pose &pose, char separator)
{
    const auto formatedPosition =
        std::format("{:.7f}{}{:.7f}{}{:.7f}", pose.position.x(), separator, pose.position.y(), separator, pose.position.z(), separator);
    const auto formatedQuaternion = std::format("{:.15f}{}{:.15f}{}{:.15f}{}{:.15f}", pose.quaternionWxyz(0), separator, pose.quaternionWxyz(1),
                                                separator, pose.quaternionWxyz(2), separator, pose.quaternionWxyz(3));
    return std::format("{}{}{}{}{}", formatedPosition, separator, formatedQuaternion, separator, opall::poseTypeToStr.at(pose.type));
}

void opall_solver_app::printPoses(const opall::PosesContainer &poses, std::ostream &outputStream, char separator)
{
    for (const auto &[poseId, pose] : poses)
    {
        outputStream << poseId << separator << poseAsString(pose, separator) << "\n";
    }
}

void opall_solver_app::printReport(const std::filesystem::path pathToFile, const ReportData &reportData, const ReportConfig &reportConfig)
{
    const std::chrono::zoned_time localTime{std::chrono::current_zone(), std::chrono::system_clock::now()};
    std::ofstream file{pathToFile};

    file << "Opall Solver App Report\n";
    file << std::format("Date and time: {:%Y-%m-%d %H:%M:%S %Z}\n", localTime);
    file << "App version: " << APP_VERSION_MAJOR << '.' << APP_VERSION_MINOR << '.' << APP_VERSION_PATCH << '\n';
    file << "\n";
    file << "Ceres solver report:\n";
    file << reportData.getSolverSummary().fullReport;
    file << "\n";
    file << "Optimized poses:\n";
    file << "poseId,x,y,z,qw,qx,qy,qz\n";
    printPoses(reportData.getOptimizationDataContainer().posesContainer, file, ',');
}
