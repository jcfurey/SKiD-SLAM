#pragma once

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace liorf::frames
{

enum class GeographicFrameMode
{
    LOCAL_ONLY,
    ECEF_ANCHORED,
};

inline GeographicFrameMode parseGeographicFrameMode(const std::string & value)
{
    if (value == "local_only")
        return GeographicFrameMode::LOCAL_ONLY;
    if (value == "ecef_anchored")
        return GeographicFrameMode::ECEF_ANCHORED;
    throw std::invalid_argument(
        "liorf.geographicFrameMode must be 'local_only' or 'ecef_anchored'");
}

// TF2 frame IDs are relative names: leading slashes are deprecated and make
// otherwise identical frame IDs compare differently. Configuration may still
// contain them, so normalize at the boundary.
inline std::string normalizeFrameId(std::string frame)
{
    const auto first = frame.find_first_not_of('/');
    if (first == std::string::npos)
        return {};
    frame.erase(0, first);

    const auto last = frame.find_last_not_of('/');
    frame.erase(last + 1);
    return frame;
}

// Local robot frames are prefixed once. A slash inside a configured frame
// means the caller supplied a fully resolved platform/site frame explicitly.
inline std::string resolveRobotFrame(
    const std::string & robot_id, const std::string & configured_frame)
{
    const std::string frame = normalizeFrameId(configured_frame);
    const std::string robot = normalizeFrameId(robot_id);
    if (frame.empty())
        throw std::invalid_argument("frame ID must not be empty");
    if (robot.empty() || frame.find('/') != std::string::npos)
        return frame;
    return robot + "/" + frame;
}

struct MapDatum
{
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double ellipsoid_height_m = 0.0;

    // Positive rotation of the map x axis from east toward north, in radians.
    // Equivalently, R_ecef_map = R_ecef_enu * Rz(map_yaw_rad).
    double map_yaw_rad = 0.0;
};

inline void validateDatum(const MapDatum & datum)
{
    if (!std::isfinite(datum.latitude_deg) ||
        !std::isfinite(datum.longitude_deg) ||
        !std::isfinite(datum.ellipsoid_height_m) ||
        !std::isfinite(datum.map_yaw_rad) ||
        datum.latitude_deg < -90.0 || datum.latitude_deg > 90.0 ||
        datum.longitude_deg < -180.0 || datum.longitude_deg > 180.0)
    {
        throw std::invalid_argument(
            "invalid WGS-84 map datum (latitude/longitude are degrees, "
            "altitude is ellipsoid height, yaw is radians)");
    }
}

// Double-precision boundary between a small local map and WGS-84 ECEF. Point
// clouds remain in the local map; only poses/anchors cross this boundary.
class GeographicMapAnchor
{
public:
    explicit GeographicMapAnchor(const MapDatum & datum)
    : datum_(datum),
      local_cartesian_(
          datum.latitude_deg, datum.longitude_deg,
          datum.ellipsoid_height_m, GeographicLib::Geocentric::WGS84())
    {
        validateDatum(datum_);

        // GeographicLib computes the optional rotation only when the caller
        // supplies an output vector with exactly nine elements.
        std::vector<double> ecef_from_enu_values(9);
        GeographicLib::Geocentric::WGS84().Forward(
            datum_.latitude_deg, datum_.longitude_deg,
            datum_.ellipsoid_height_m,
            ecef_origin_.x(), ecef_origin_.y(), ecef_origin_.z(),
            ecef_from_enu_values);

        if (ecef_from_enu_values.size() != 9)
            throw std::runtime_error("GeographicLib returned an invalid ENU rotation");

        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
            ecef_from_enu(ecef_from_enu_values.data());
        const Eigen::Matrix3d enu_from_map =
            Eigen::AngleAxisd(datum_.map_yaw_rad, Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        ecef_from_map_ = ecef_from_enu * enu_from_map;
        map_from_enu_ = enu_from_map.transpose();
    }

    const MapDatum & datum() const { return datum_; }
    const Eigen::Vector3d & ecefOrigin() const { return ecef_origin_; }
    const Eigen::Matrix3d & ecefFromMapRotation() const
    {
        return ecef_from_map_;
    }

    Eigen::Isometry3d earthFromMap() const
    {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = ecef_from_map_;
        result.translation() = ecef_origin_;
        return result;
    }

    Eigen::Vector3d mapFromGeodetic(
        const double latitude_deg, const double longitude_deg,
        const double ellipsoid_height_m) const
    {
        Eigen::Vector3d enu;
        local_cartesian_.Forward(
            latitude_deg, longitude_deg, ellipsoid_height_m,
            enu.x(), enu.y(), enu.z());
        return map_from_enu_ * enu;
    }

private:
    MapDatum datum_;
    GeographicLib::LocalCartesian local_cartesian_;
    Eigen::Vector3d ecef_origin_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d ecef_from_map_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d map_from_enu_ = Eigen::Matrix3d::Identity();
};

inline Eigen::Isometry3d mapFromOdom(
    const Eigen::Isometry3d & map_from_body,
    const Eigen::Isometry3d & odom_from_body)
{
    return map_from_body * odom_from_body.inverse();
}

struct UtmUpsCoordinate
{
    int zone = GeographicLib::UTMUPS::INVALID;
    bool north = true;
    double easting_m = 0.0;
    double northing_m = 0.0;
    double convergence_deg = 0.0;
    double scale = 1.0;

    std::string frameId() const
    {
        if (zone == GeographicLib::UTMUPS::UPS)
            return std::string("ups_") + (north ? "N" : "S");
        if (zone < GeographicLib::UTMUPS::MINUTMZONE ||
            zone > GeographicLib::UTMUPS::MAXUTMZONE)
            throw std::logic_error("invalid UTM/UPS zone");
        return "utm_" + std::to_string(zone) + (north ? "N" : "S");
    }
};

inline UtmUpsCoordinate projectToUtmUps(
    const double latitude_deg, const double longitude_deg)
{
    UtmUpsCoordinate result;
    GeographicLib::UTMUPS::Forward(
        latitude_deg, longitude_deg, result.zone, result.north,
        result.easting_m, result.northing_m, result.convergence_deg,
        result.scale);
    return result;
}

}  // namespace liorf::frames
