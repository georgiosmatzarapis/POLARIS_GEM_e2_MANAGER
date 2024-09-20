// author: georgiosmatzarapis

#pragma once

#include <std_msgs/String.h>

#include "core/publisher.hpp"
#include "core/subscriber.hpp"
#include "gem_planner/controllers/path_tracking_controller.hpp"

namespace gem_planner {

struct Waypoint final {
  std::string x{};
  std::string y{};
  std::string yaw{};

  Waypoint(std::string xVal, std::string yVal, std::string yawVal);
};

class Planner final {
 public:
  explicit Planner(ros::NodeHandle& nodeHandle, std::string waypointsFile,
                   std::string controllerType);

  void startPublishing();

 private:
  std::unique_ptr<core::Subscriber<std_msgs::String>> robotStateSubscriber_;
  std::unique_ptr<core::Publisher<gem_manager::WaypointsBatch>>
      waypointPublisher_;
  std::vector<Waypoint> waypoints_{};
  bool isHealthy_{false};
  ros::Time lastErrorTime_{ros::Time(0)};
  std::unique_ptr<PathTrackingController> pathTrackingController_{nullptr};
  ros::Rate waypointPublishingRate_{1};

  void robotStateCallback(const std_msgs::String::ConstPtr& msg);
  void loadWaypoints(const std::string& filePath);
  void publishWaypoints();

  /// @brief Duration of time to wait after an error before resuming waypoint
  /// publishing. If no new errors are detected within this time frame, waypoint
  /// publishing will continue.
  static constexpr float RECOVERY_TIME{2.0f};
};

} // namespace gem_planner
