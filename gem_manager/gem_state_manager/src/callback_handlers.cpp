// author: georgiosmatzarapis

#include "gem_state_manager/callback_handlers.hpp"
#include "gem_state_manager/state_manager.hpp"
#include "ros1_lib/subscriber.hpp"

namespace gem_state_manager {

// PUBLIC API

CallbackHandlers::CallbackHandlers(ros::NodeHandle& nodeHandle)
    : nodeHandle_{nodeHandle},
      batterySubscriber_{
          std::make_unique<ros1_lib::Subscriber<std_msgs::Float32>>(
              nodeHandle_)},
      temperatureSubscriber_{
          std::make_unique<ros1_lib::Subscriber<std_msgs::Float32>>(
              nodeHandle_)},
      gpsAccuracySubscriber_{
          std::make_unique<ros1_lib::Subscriber<std_msgs::Float32>>(
              nodeHandle_)},
      signalSubscriber_{
          std::make_unique<ros1_lib::Subscriber<std_msgs::Int32>>(nodeHandle_)},
      emergencyButtonSubscriber_{
          std::make_unique<ros1_lib::Subscriber<std_msgs::Bool>>(nodeHandle_)},
      stateManagerPtr_{nullptr} {}

void CallbackHandlers::initSubscriptions(StateManager& stateManager) {
  stateManagerPtr_ = &stateManager;

  batterySubscriber_->subscribe("/gem_manager/battery_level", 1000,
                                [this](const std_msgs::Float32::ConstPtr& msg) {
                                  this->batteryCallback(msg);
                                });

  temperatureSubscriber_->subscribe(
      "/gem_manager/temperature", 1000,
      [this](const std_msgs::Float32::ConstPtr& msg) {
        this->temperatureCallback(msg);
      });

  gpsAccuracySubscriber_->subscribe(
      "/gem_manager/gps_accuracy", 1000,
      [this](const std_msgs::Float32::ConstPtr& msg) {
        this->gpsAccuracyCallback(msg);
      });

  signalSubscriber_->subscribe("/gem_manager/signal_strength", 1000,
                               [this](const std_msgs::Int32::ConstPtr& msg) {
                                 this->signalCallback(msg);
                               });

  emergencyButtonSubscriber_->subscribe(
      "/gem_manager/emergency_button", 1000,
      [this](const std_msgs::Bool::ConstPtr& msg) {
        this->emergencyButtonCallback(msg);
      });
}

// PRIVATE API

// using enum Event; // C++ 20 onwards

void CallbackHandlers::batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stateManagerMutex_);
  if (msg->data <= BATTERY_LIMIT) {
    stateManagerPtr_->handleEvent(Event::BATTERY_LOW,
                                  std::to_string(msg->data));
  }
}

void CallbackHandlers::temperatureCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stateManagerMutex_);
  if (msg->data >= TEMPERATURE_LIMIT) {
    stateManagerPtr_->handleEvent(Event::TEMPERATURE_HIGH,
                                  std::to_string(msg->data));
  }
}

void CallbackHandlers::gpsAccuracyCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stateManagerMutex_);
  static ros::Time lastErrorTime{ros::Time::now()};
  static bool gpsErrorActive{false};

  if (msg->data > GPS_LIMIT) {
    if (!gpsErrorActive) {
      lastErrorTime = ros::Time::now();
      gpsErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() > 15.0) {
      stateManagerPtr_->handleEvent(Event::GPS_INACCURATE,
                                    std::to_string(msg->data));
    }
  } else {
    gpsErrorActive = false;
  }
}

void CallbackHandlers::signalCallback(const std_msgs::Int32::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stateManagerMutex_);
  static ros::Time lastErrorTime{ros::Time::now()};
  static bool signalErrorActive{false};

  if (msg->data == SIGNAL_LOST_LIMIT) {
    if (!signalErrorActive) {
      lastErrorTime = ros::Time::now();
      signalErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() >= 10.0) {
      stateManagerPtr_->handleEvent(Event::SIGNAL_LOST,
                                    std::to_string(msg->data));
    }
  } else if (msg->data == SIGNAL_LOW_LIMIT) {
    if (!signalErrorActive) {
      lastErrorTime = ros::Time::now();
      signalErrorActive = true;
    } else if ((ros::Time::now() - lastErrorTime).toSec() >= 20.0) {
      stateManagerPtr_->handleEvent(Event::SIGNAL_LOW,
                                    std::to_string(msg->data));
    }
  } else {
    signalErrorActive = false;
  }
}

void CallbackHandlers::emergencyButtonCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(stateManagerMutex_);
  if (msg->data) {
    stateManagerPtr_->handleEvent(Event::EMERGENCY, std::to_string(msg->data));
  }
}

} // namespace gem_state_manager
