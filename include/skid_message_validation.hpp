#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "liorf/msg/context_info.hpp"

namespace liorf::messages {

inline bool validCloud(const sensor_msgs::msg::PointCloud2& cloud) {
  const std::uint64_t row_bytes = std::uint64_t(cloud.width) * cloud.point_step;
  const std::uint64_t total_bytes = std::uint64_t(cloud.height) * cloud.row_step;
  if (row_bytes > cloud.row_step || total_bytes != cloud.data.size() ||
      (cloud.width > 0 && (cloud.point_step == 0 || cloud.height == 0)))
    return false;
  for (const auto& field : cloud.fields) {
    const unsigned sizes[] = {0, 1, 1, 2, 2, 4, 4, 4, 8};
    if (field.datatype == 0 || field.datatype > 8 || field.count == 0 ||
        std::uint64_t(field.offset) + std::uint64_t(field.count) * sizes[field.datatype] > cloud.point_step)
      return false;
  }
  return true;
}

inline bool validContext(const liorf::msg::ContextInfo& message,
                         int rings, int sectors, bool solid) {
  const auto finite = [](const auto& values) {
    return std::all_of(values.begin(), values.end(), [](float value) { return std::isfinite(value); });
  };
  const float pose[] = {message.pose_x, message.pose_y, message.pose_z,
                       message.pose_roll, message.pose_pitch, message.pose_yaw};
  if (rings <= 0 || sectors <= 0 || message.num_ring != rings || message.num_sector != sectors ||
      message.robot_id.empty() || message.keyframe_index < 0 ||
      message.keyframe_index > std::numeric_limits<std::uint32_t>::max() ||
      !std::all_of(std::begin(pose), std::end(pose), [](float value) { return std::isfinite(value); }) ||
      !validCloud(message.scan_cloud))
    return false;
  if (solid) {
    return message.rsolid.size() == static_cast<std::size_t>(rings) &&
        message.asolid.size() == static_cast<std::size_t>(sectors) &&
        finite(message.rsolid) && finite(message.asolid) &&
        std::any_of(message.rsolid.begin(), message.rsolid.end(), [](float value) { return value != 0; });
  }
  return message.ring_key.size() == static_cast<std::size_t>(rings) &&
      message.scan_context_bin.size() == std::size_t(rings) * sectors &&
      finite(message.ring_key) && finite(message.scan_context_bin);
}

}  // namespace liorf::messages
