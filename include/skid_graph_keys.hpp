#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

namespace liorf::graph_keys {

// Keep locally owned trajectory poses in their own explicit symbol space.
// Remote poses will coexist in the same Values object during Equation (6)
// optimization, so a bare keyframe index is no longer a safe graph key.
inline gtsam::Key localPose(std::size_t keyframe_index) {
  return gtsam::Symbol('x', static_cast<std::uint64_t>(keyframe_index));
}

inline bool isLocalPose(gtsam::Key key) noexcept {
  return gtsam::Symbol(key).chr() == 'x';
}

}  // namespace liorf::graph_keys
