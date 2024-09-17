// author: georgiosmatzarapis

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace gem_planner {

struct Waypoint final {
  double x;
  double y;
  double yaw;
};

class Planner final {
 public:
  explicit Planner(ros::NodeHandle& nodeHandle,
                   const std::string& waypointsFile);

  void startPublishing();

 private:
  ros::Subscriber robotStateSubscriber_;
  ros::Publisher waypointPublisher_;
  std::vector<Waypoint> waypoints_{};
  std::size_t waypointsSize_{};
  bool isHealthy_{false};
  std::size_t currentLocation_{0};
  ros::Time lastErrorTime_{ros::Time(0)};
  ros::Rate waypointPublishingRate_{20};

  void robotStateCallback(const std_msgs::String::ConstPtr& msg);
  void loadWaypoints(const std::string& filePath);
  void publishWaypoints();

  /// @brief Duration of time to wait after an error before resuming waypoint
  /// publishing. If no new errors are detected within this time frame, waypoint
  /// publishing will continue.
  static constexpr float RECOVERY_TIME{5.0f};
};

} // namespace gem_planner
