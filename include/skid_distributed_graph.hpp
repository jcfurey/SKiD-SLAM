#pragma once

#include <memory>
#include <gtsam/nonlinear/ISAM2.h>
#include "skid_graph_sync.hpp"

namespace liorf::graph_sync {

// Rebuild only on a changed distributed state, with unchanged local factors.
// A fresh solver provides transactionality and removes orphan peer variables
// when their final registration is withdrawn. The caller swaps it in only
// after successful optimization; failed updates leave the live solver intact.
std::unique_ptr<gtsam::ISAM2> rebuild(
    const gtsam::ISAM2& current,
    const std::vector<Constraint>& constraints,
    const PoseStore& corrections,
    const std::string& local_robot,
    const gtsam::SharedNoiseModel& remote_motion_noise);

}  // namespace liorf::graph_sync
