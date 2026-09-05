#pragma once

#include <optional>
#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace liorf::map_alignment {
struct Edge {
  int from;
  int to;
  gtsam::Pose3 measurement;
};

// Solve only the component connected to the configured anchor. Detached maps
// have no alignment; seeding them at identity creates a fictitious relation.
inline std::optional<gtsam::Pose3> solve(
    const std::vector<Edge>& edges, int anchor, int local) {
  gtsam::Values initial;
  initial.insert(anchor, gtsam::Pose3());
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto& edge : edges) {
      if (initial.exists(edge.from) && !initial.exists(edge.to)) {
        initial.insert(edge.to, initial.at<gtsam::Pose3>(edge.from) * edge.measurement);
        changed = true;
      } else if (!initial.exists(edge.from) && initial.exists(edge.to)) {
        initial.insert(edge.from, initial.at<gtsam::Pose3>(edge.to) * edge.measurement.inverse());
        changed = true;
      }
    }
  }
  if (!initial.exists(local)) return std::nullopt;
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      anchor, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1.0e-4));
  for (const auto& edge : edges) {
    if (initial.exists(edge.from) && initial.exists(edge.to))
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          edge.from, edge.to, edge.measurement,
          gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
  }
  return gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize().at<gtsam::Pose3>(local);
}
}  // namespace liorf::map_alignment
