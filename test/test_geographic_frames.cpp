#include "geographic_frames.hpp"

#include <GeographicLib/Geocentric.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace
{

using liorf::frames::GeographicMapAnchor;
using liorf::frames::MapDatum;

TEST(FrameNames, ResolvesRobotLocalFramesExactlyOnce)
{
    EXPECT_EQ(liorf::frames::normalizeFrameId("//earth/"), "earth");
    EXPECT_EQ(liorf::frames::resolveRobotFrame("jackal0", "odom"),
              "jackal0/odom");
    EXPECT_EQ(liorf::frames::resolveRobotFrame("jackal0", "/jackal0/odom"),
              "jackal0/odom");
    EXPECT_EQ(liorf::frames::resolveRobotFrame("jackal0", "site_alpha/map"),
              "site_alpha/map");
    EXPECT_THROW(liorf::frames::resolveRobotFrame("jackal0", "/"),
                 std::invalid_argument);
}

TEST(GeographicAnchor, EquatorialDatumUsesEcefAndEnuAxes)
{
    const GeographicMapAnchor anchor(MapDatum{});
    const Eigen::Isometry3d earth_from_map = anchor.earthFromMap();

    EXPECT_NEAR(earth_from_map.translation().x(), 6378137.0, 1e-6);
    EXPECT_NEAR(earth_from_map.translation().y(), 0.0, 1e-9);
    EXPECT_NEAR(earth_from_map.translation().z(), 0.0, 1e-9);

    // At latitude/longitude zero: east is +ECEF Y, north is +ECEF Z,
    // and up is +ECEF X.
    EXPECT_TRUE(earth_from_map.rotation().col(0).isApprox(
        Eigen::Vector3d(0.0, 1.0, 0.0), 1e-14));
    EXPECT_TRUE(earth_from_map.rotation().col(1).isApprox(
        Eigen::Vector3d(0.0, 0.0, 1.0), 1e-14));
    EXPECT_TRUE(earth_from_map.rotation().col(2).isApprox(
        Eigen::Vector3d(1.0, 0.0, 0.0), 1e-14));
    EXPECT_TRUE(anchor.mapFromGeodetic(0.0, 0.0, 0.0).isZero(1e-9));
}

TEST(GeographicAnchor, LocalMapPoseComposesBackToExactEcefPoint)
{
    MapDatum datum;
    datum.latitude_deg = 34.12;
    datum.longitude_deg = -90.55;
    datum.ellipsoid_height_m = 73.4;
    datum.map_yaw_rad = 0.37;
    const GeographicMapAnchor anchor(datum);

    constexpr double point_latitude = 34.1213;
    constexpr double point_longitude = -90.5471;
    constexpr double point_height = 81.2;
    const Eigen::Vector3d map_point = anchor.mapFromGeodetic(
        point_latitude, point_longitude, point_height);
    const Eigen::Vector3d composed_ecef = anchor.earthFromMap() * map_point;

    Eigen::Vector3d direct_ecef;
    GeographicLib::Geocentric::WGS84().Forward(
        point_latitude, point_longitude, point_height,
        direct_ecef.x(), direct_ecef.y(), direct_ecef.z());
    EXPECT_TRUE(composed_ecef.isApprox(direct_ecef, 1e-6));
}

TEST(GeographicAnchor, DistantMapsKeepDistinctEcefOrigins)
{
    MapDatum mississippi;
    mississippi.latitude_deg = 34.0;
    mississippi.longitude_deg = -90.0;
    MapDatum california;
    california.latitude_deg = 34.0;
    california.longitude_deg = -118.0;

    const GeographicMapAnchor map_a(mississippi);
    const GeographicMapAnchor map_b(california);
    EXPECT_GT((map_a.ecefOrigin() - map_b.ecefOrigin()).norm(), 2.0e6);
}

TEST(UtmProjection, FrameIdentityIncludesZoneAndHemisphere)
{
    const auto mississippi = liorf::frames::projectToUtmUps(34.0, -89.0);
    const auto california = liorf::frames::projectToUtmUps(34.0, -118.0);
    const auto southern = liorf::frames::projectToUtmUps(-34.0, 151.0);

    EXPECT_EQ(mississippi.frameId(), "utm_16N");
    EXPECT_EQ(california.frameId(), "utm_11N");
    EXPECT_EQ(southern.frameId(), "utm_56S");
    EXPECT_NE(mississippi.frameId(), california.frameId());
    EXPECT_TRUE(std::isfinite(mississippi.easting_m));
    EXPECT_TRUE(std::isfinite(mississippi.northing_m));
}

TEST(FrameCorrection, CorrectedAndContinuousPosesDefineMapToOdom)
{
    Eigen::Isometry3d map_from_body = Eigen::Isometry3d::Identity();
    map_from_body.translation() = Eigen::Vector3d(12.0, -3.0, 0.5);
    map_from_body.linear() =
        Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Isometry3d odom_from_body = Eigen::Isometry3d::Identity();
    odom_from_body.translation() = Eigen::Vector3d(10.0, -1.0, 0.5);
    odom_from_body.linear() =
        Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const Eigen::Isometry3d map_from_odom =
        liorf::frames::mapFromOdom(map_from_body, odom_from_body);
    EXPECT_TRUE((map_from_odom * odom_from_body).matrix().isApprox(
        map_from_body.matrix(), 1e-12));
}

}  // namespace
