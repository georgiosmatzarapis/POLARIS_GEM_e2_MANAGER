// author: georgiosmatzarapis

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

namespace gem_state_manager {

class StateManager;

class CallbackHandlers final {
 public:
  explicit CallbackHandlers(ros::NodeHandle& nodeHandle);

  void initSubscriptions(StateManager& stateManager);

 private:
  ros::NodeHandle& nodeHandle_;
  StateManager* stateManagerPtr_;
  ros::Subscriber batterySubscriber_;
  ros::Subscriber temperatureSubscriber_;
  ros::Subscriber gpsAccuracySubscriber_;
  ros::Subscriber signalSubscriber_;
  ros::Subscriber emergencyButtonSubscriber_;
  std::mutex stateManagerMutex_;

  void batteryCallback(const std_msgs::Float32::ConstPtr& msg);
  void temperatureCallback(const std_msgs::Float32::ConstPtr& msg);
  void gpsAccuracyCallback(const std_msgs::Float32::ConstPtr& msg);
  void signalCallback(const std_msgs::Int32::ConstPtr& msg);
  void emergencyButtonCallback(const std_msgs::Bool::ConstPtr& msg);

  static constexpr float BATTERY_LIMIT{50.0f};
  static constexpr float TEMPERATURE_LIMIT{55.0f};
  static constexpr float GPS_LIMIT{200.0f};
  static constexpr int SIGNAL_LOST_LIMIT{0};
  static constexpr int SIGNAL_LOW_LIMIT{2};
};

} // namespace gem_state_manager
