#pragma once

#include <Eigen/Core>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <variant>

namespace opall
{
// using PoseId = std::string;
using PoseId = std::int64_t;
using PointId = std::int64_t;

using PosesIds = std::unordered_set<PoseId>;
using Position = Eigen::Vector3d;
using QuaternionWxyz = Eigen::Vector4d;

namespace uncertainty3d
{
using StandardDeviation = Eigen::Vector3d;

struct CovarianceMatrix
{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matrix;
};

struct InformationMatrix
{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matrix;
};

struct SquareRootInformationMatrix
{
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> matrix;
};

using Uncertainty = std::variant<StandardDeviation, CovarianceMatrix, InformationMatrix, SquareRootInformationMatrix>;
} // namespace uncertainty3d

struct Pose
{
    enum class Type
    {
        FREE,
        FIXED
    };
    Position position{0.0, 0.0, 0.0};
    QuaternionWxyz quaternionWxyz{1.0, 0.0, 0.0, 0.0};
    Type type{Type::FREE};
};

static const std::unordered_map<std::string, Pose::Type> poseTypeStrToPoseType = {{"free", Pose::Type::FREE}, {"fixed", Pose::Type::FIXED}};

static const std::unordered_map<Pose::Type, std::string> poseTypeToStr = {{Pose::Type::FREE, "free"}, {Pose::Type::FIXED, "fixed"}};

using PosesContainer = std::unordered_map<PoseId, Pose>;

namespace point_3d
{
using Point = Eigen::Vector3d;
using PointWithUncertainty = std::pair<Point, uncertainty3d::Uncertainty>;
} // namespace point_3d

using PointObservations = std::unordered_map<PointId, point_3d::PointWithUncertainty>;
using PointObservationsContainer = std::map<PoseId, PointObservations>;
using FixedPosesSet = std::unordered_set<PoseId>;
using PointContainer = std::unordered_map<PointId, point_3d::Point>;
// using ControlPointContainer = std::unordered_map<PointId, point_3d::PointWithUncertainty>;

} // namespace opall