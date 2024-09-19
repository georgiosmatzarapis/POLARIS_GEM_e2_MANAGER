// author: georgiosmatzarapis

#include "gem_planner/controllers/stanley.hpp"

namespace gem_planner {

// PUBLIC API

Stanley::Stanley(const std::vector<Waypoint>& waypoints) noexcept
    : waypoints_{waypoints},
      waypointsSize_{waypoints.size()} {}

gem_manager::WaypointsBatch Stanley::update() {
  gem_manager::WaypointsBatch waypointsBatchMsg;
  std::size_t batchEnd{std::min(currentLocation_ + BATCH_SIZE, waypointsSize_)};

  for (std::size_t wptIndex{currentLocation_}; wptIndex < batchEnd;
       ++wptIndex) {
    gem_manager::Waypoint waypointMsg;
    waypointMsg.x = waypoints_[wptIndex].x;
    waypointMsg.y = waypoints_[wptIndex].y;
    waypointMsg.yaw = waypoints_[wptIndex].yaw;
    waypointsBatchMsg.waypoints.push_back(waypointMsg);
  }

  if (batchEnd >= waypointsSize_) {
    currentLocation_ = 0;
  } else {
    currentLocation_ = batchEnd;
  }

  return waypointsBatchMsg;
}

std::uint8_t Stanley::getWaypointPublishingRate() const noexcept {
  return WAYPOINT_PUBLISHING_RATE;
}

} // namespace gem_planner
