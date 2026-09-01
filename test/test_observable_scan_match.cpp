#include "observable_scan_match.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace observable = liorf::observable_scan_match;

TEST(ObservableScanMatch, TernaryBandHasHeldPartialAndFullDirections)
{
    const observable::GateConfig config{0.005, 0.02, 0.0};
    const Eigen::Matrix3d information =
        (Eigen::Vector3d{0.001, 0.019, 0.98}).asDiagonal();

    const observable::BlockObservability result = observable::analyze(information, config);

    ASSERT_TRUE(result.has_information);
    EXPECT_TRUE(result.constrained());
    EXPECT_NEAR(result.information_shares(0), 0.001, 1.0e-12);
    EXPECT_NEAR(result.information_shares(1), 0.019, 1.0e-12);
    EXPECT_NEAR(result.information_shares(2), 0.98, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.admit_fractions(0), 0.0);
    EXPECT_NEAR(result.admit_fractions(1), 14.0 / 15.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.admit_fractions(2), 1.0);
}

TEST(ObservableScanMatch, InformationSharesAreInvariantToMeasurementScale)
{
    const observable::GateConfig config{0.005, 0.02, 0.0};
    Eigen::AngleAxisd rotation(0.37, Eigen::Vector3d{1.0, 2.0, 3.0}.normalized());
    const Eigen::Matrix3d basis = rotation.toRotationMatrix();
    const Eigen::Matrix3d information =
        basis * (Eigen::Vector3d{0.002, 0.018, 0.98}).asDiagonal() * basis.transpose();

    const auto first = observable::analyze(information, config);
    const auto scaled = observable::analyze(173.0 * information, config);

    ASSERT_TRUE(first.has_information);
    ASSERT_TRUE(scaled.has_information);
    EXPECT_TRUE(first.information_shares.isApprox(scaled.information_shares, 1.0e-12));
    EXPECT_TRUE(first.admit_fractions.isApprox(scaled.admit_fractions, 1.0e-12));
}

TEST(ObservableScanMatch, PartialAdmissionHonorsOneScanBudget)
{
    const observable::GateConfig config{0.005, 0.02, 0.1};
    const Eigen::Matrix3d information =
        (Eigen::Vector3d{0.001, 0.019, 0.98}).asDiagonal();
    const auto observability = observable::analyze(information, config);
    double used = 0.0;

    const Eigen::Vector3d first = observable::apply(
        Eigen::Vector3d{0.2, 0.3, 0.4}, observability, config, &used);
    const Eigen::Vector3d second = observable::apply(
        Eigen::Vector3d{0.2, 0.3, 0.4}, observability, config, &used);

    EXPECT_TRUE(first.isApprox(Eigen::Vector3d{0.0, 0.1, 0.4}, 1.0e-12));
    EXPECT_TRUE(second.isApprox(Eigen::Vector3d{0.0, 0.0, 0.4}, 1.0e-12));
    EXPECT_NEAR(used, 0.1, 1.0e-12);
}

TEST(ObservableScanMatch, DisabledGateLeavesCorrectionUnchanged)
{
    const observable::GateConfig config{0.0, 0.0, 0.0};
    const Eigen::Matrix3d information =
        (Eigen::Vector3d{0.0, 0.0, 1.0}).asDiagonal();
    const auto observability = observable::analyze(information, config);
    const Eigen::Vector3d correction{1.0, -2.0, 3.0};

    EXPECT_FALSE(observability.constrained());
    EXPECT_TRUE(observable::apply(correction, observability, config, nullptr)
                    .isApprox(correction, 0.0));
}

TEST(ObservableScanMatch, RejectsMisorderedOrNonfiniteConfiguration)
{
    EXPECT_FALSE((observable::GateConfig{0.02, 0.005, 0.0}).valid());
    EXPECT_FALSE((observable::GateConfig{0.0, 1.1, 0.0}).valid());
    EXPECT_FALSE((observable::GateConfig{0.0, 0.02, -0.1}).valid());
    EXPECT_TRUE((observable::GateConfig{0.005, 0.02, 0.1}).valid());
}
