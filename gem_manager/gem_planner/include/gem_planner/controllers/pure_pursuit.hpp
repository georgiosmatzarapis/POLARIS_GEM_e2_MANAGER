// author: georgiosmatzarapis

#pragma once

#include "gem_planner/controllers/path_tracking_controller.hpp"
#include "gem_planner/planner.hpp"

namespace gem_planner {

class PurePursuit final : public PathTrackingController {
 public:
  PurePursuit(const PurePursuit&) = delete;
  PurePursuit& operator=(const PurePursuit&) = delete;
  PurePursuit(PurePursuit&&) noexcept = delete;
  PurePursuit& operator=(PurePursuit&&) noexcept = delete;
  ~PurePursuit() override = default;

  explicit PurePursuit(const std::vector<Waypoint>& waypoints) noexcept;

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
