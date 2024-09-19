// author: georgiosmatzarapis

#pragma once

#include <ros/ros.h>

#include <gem_manager/WaypointsBatch.h>

namespace gem_planner {

class PathTrackingController {
 public:
  virtual gem_manager::WaypointsBatch update() = 0;
  [[nodiscard]] virtual std::uint8_t
  getWaypointPublishingRate() const noexcept = 0;
  virtual ~PathTrackingController() = default;
};

} // namespace gem_planner
