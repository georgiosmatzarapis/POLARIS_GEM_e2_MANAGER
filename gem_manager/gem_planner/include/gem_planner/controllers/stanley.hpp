// author: georgiosmatzarapis

#pragma once

#include "gem_planner/controllers/path_tracking_controller.hpp"
#include "gem_planner/planner.hpp"

namespace gem_planner {

class Stanley final : public PathTrackingController {
 public:
  Stanley(const Stanley&) = delete;
  Stanley& operator=(const Stanley&) = delete;
  Stanley(Stanley&&) noexcept = delete;
  Stanley& operator=(Stanley&&) noexcept = delete;
  ~Stanley() override = default;

  explicit Stanley(const std::vector<Waypoint>& waypoints) noexcept;

  gem_manager::WaypointsBatch update() override;
  std::uint8_t getWaypointPublishingRate() const noexcept override;

 private:
  const std::vector<Waypoint>& waypoints_;
  std::size_t waypointsSize_;
  std::size_t currentLocation_{0};

  /// @brief Batch size for publishing waypoints
  static constexpr std::size_t BATCH_SIZE{40};
  static constexpr std::uint8_t WAYPOINT_PUBLISHING_RATE{6};
};

} // namespace gem_planner
