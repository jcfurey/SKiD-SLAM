#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace liorf::observable_scan_match
{

// A scale-free, block-local gate for point-to-plane pose information. The
// minimum/full values are shares of the block trace, so translation and
// rotation are never compared despite carrying different units.
struct GateConfig
{
    double minimum_information_share = 0.0;
    double full_information_share = 0.0;
    double partial_admission_budget = 0.0;

    [[nodiscard]] bool enabled() const
    {
        return full_information_share > 0.0;
    }

    [[nodiscard]] bool valid() const
    {
        return std::isfinite(minimum_information_share) &&
               std::isfinite(full_information_share) &&
               std::isfinite(partial_admission_budget) &&
               minimum_information_share >= 0.0 &&
               full_information_share >= minimum_information_share &&
               full_information_share <= 1.0 &&
               partial_admission_budget >= 0.0;
    }
};

struct BlockObservability
{
    Eigen::Vector3d information_shares = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
    Eigen::Vector3d admit_fractions = Eigen::Vector3d::Ones();
    bool has_information = false;

    [[nodiscard]] bool constrained(double tolerance = 1.0e-9) const
    {
        return (admit_fractions.array() < 1.0 - tolerance).any();
    }
};

inline double admitFraction(double information_share, const GateConfig & config)
{
    if (!config.enabled())
        return 1.0;

    if (!std::isfinite(information_share) || information_share <= config.minimum_information_share)
        return 0.0;
    if (information_share >= config.full_information_share)
        return 1.0;
    if (!(config.minimum_information_share < config.full_information_share))
        return 0.0;

    return (information_share - config.minimum_information_share) /
           (config.full_information_share - config.minimum_information_share);
}

inline BlockObservability analyze(const Eigen::Matrix3d & information,
                                  const GateConfig & config)
{
    BlockObservability result;
    const Eigen::Matrix3d symmetric = 0.5 * (information + information.transpose());
    const double trace = symmetric.trace();
    if (!symmetric.allFinite() || !std::isfinite(trace) ||
        trace <= std::numeric_limits<double>::epsilon())
    {
        if (config.enabled())
            result.admit_fractions.setZero();
        return result;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    if (solver.info() != Eigen::Success)
    {
        if (config.enabled())
            result.admit_fractions.setZero();
        return result;
    }

    result.has_information = true;
    result.eigenvectors = solver.eigenvectors();
    result.information_shares = (solver.eigenvalues() / trace).cwiseMax(0.0);
    for (Eigen::Index index = 0; index < 3; ++index)
        result.admit_fractions(index) = admitFraction(result.information_shares(index), config);
    return result;
}

// Retain the full correction on observable directions, hold the prediction on
// non-observable directions, and linearly blend the two in the partial band.
// The optional budget bounds cumulative partial-band authority over one scan;
// fully observable directions are deliberately not clipped.
inline Eigen::Vector3d apply(const Eigen::Vector3d & unconstrained_correction,
                             const BlockObservability & observability,
                             const GateConfig & config,
                             double * partial_budget_used)
{
    if (!config.enabled())
        return unconstrained_correction;

    Eigen::Vector3d components =
        observability.eigenvectors.transpose() * unconstrained_correction;
    for (Eigen::Index index = 0; index < 3; ++index)
    {
        const double admit = std::clamp(observability.admit_fractions(index), 0.0, 1.0);
        double admitted_component = admit * components(index);

        if (admit > 0.0 && admit < 1.0 &&
            config.partial_admission_budget > 0.0 && partial_budget_used != nullptr)
        {
            const double remaining = std::max(
                0.0, config.partial_admission_budget - *partial_budget_used);
            admitted_component = std::clamp(admitted_component, -remaining, remaining);
            *partial_budget_used += std::abs(admitted_component);
        }
        components(index) = admitted_component;
    }
    return observability.eigenvectors * components;
}

}  // namespace liorf::observable_scan_match
