// author: georgiosmatzarapis

#include "gem_state_manager/callback_handlers.hpp"
#include "gem_state_manager/state_manager.hpp"

namespace gem_state_manager {

// PUBLIC API

CallbackHandlers::CallbackHandlers(ros::NodeHandle& nodeHandle)
    : nodeHandle_{nodeHandle},
      stateManagerPtr_{nullptr} {}

void CallbackHandlers::initSubscriptions(StateManager& stateManager) {
  stateManagerPtr_ = &stateManager;

  batterySubscriber_ = nodeHandle_.subscribe(
      "/mock/battery_level", 1000, &CallbackHandlers::batteryCallback, this);

  temperatureSubscriber_ = nodeHandle_.subscribe(
      "/mock/temperature", 1000, &CallbackHandlers::temperatureCallback, this);

  gpsAccuracySubscriber_ = nodeHandle_.subscribe(
      "/mock/gps_accuracy", 1000, &CallbackHandlers::gpsAccuracyCallback, this);

  signalSubscriber_ = nodeHandle_.subscribe(
      "/mock/signal_strength", 1000, &CallbackHandlers::signalCallback, this);

  emergencyButtonSubscriber_ =
      nodeHandle_.subscribe("/mock/emergency_button", 1000,
                            &CallbackHandlers::emergencyButtonCallback, this);
}

// PRIVATE API

using enum StateManager::Event;

void CallbackHandlers::batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data <= BATTERY_LIMIT) {
    stateManagerPtr_->handleEvent(BATTERY_LOW, std::to_string(msg->data));
  }
}

void CallbackHandlers::temperatureCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  if (msg->data >= TEMPERATURE_LIMIT) {
    stateManagerPtr_->handleEvent(TEMPERATURE_HIGH, std::to_string(msg->data));
  }
}

void CallbackHandlers::gpsAccuracyCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  static ros::Time lastErrorTime{ros::Time::now()};
  static bool gpsErrorActive{false};

  if (msg->data < GPS_LIMIT) {
    if (!gpsErrorActive) {
      lastErrorTime = ros::Time::now();
      gpsErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() > 15.0) {
      stateManagerPtr_->handleEvent(GPS_INACCURATE, std::to_string(msg->data));
    }
  } else {
    gpsErrorActive = false;
  }
}

void CallbackHandlers::signalCallback(const std_msgs::Int32::ConstPtr& msg) {
  static ros::Time lastErrorTime{ros::Time::now()};
  static bool signalErrorActive{false};

  if (msg->data == SIGNAL_LOST_LIMIT) {
    if (!signalErrorActive) {
      lastErrorTime = ros::Time::now();
      signalErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() >= 10.0) {
      stateManagerPtr_->handleEvent(SIGNAL_LOST, std::to_string(msg->data));
    }
  } else if (msg->data == SIGNAL_LOW_LIMIT) {
    if (!signalErrorActive) {
      lastErrorTime = ros::Time::now();
      signalErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() >= 20.0) {
      stateManagerPtr_->handleEvent(SIGNAL_LOW, std::to_string(msg->data));
    }
  } else {
    signalErrorActive = false;
  }
}

void CallbackHandlers::emergencyButtonCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data) {
    stateManagerPtr_->handleEvent(EMERGENCY, std::to_string(msg->data));
  }
}

} // namespace gem_state_manager
