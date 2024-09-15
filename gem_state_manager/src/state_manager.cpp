// author: georgiosmatzarapis

#include <ros/ros.h>

#include "gem_state_manager/callback_handlers.hpp"
#include "gem_state_manager/state_manager.hpp"

namespace gem_state_manager {

// PUBLIC API

StateManager::StateManager(CallbackHandlers& callbackHandlers) {
  callbackHandlers.initSubscriptions(*this);
}

void StateManager::handleEvent(Event event, const std::string& data) {
  std::lock_guard<std::mutex> lock(stateMutex_);
  using enum RobotState;

  ROS_INFO("Event received: [%s], value: [%s]", eventToString(event).c_str(),
           data.c_str());

  switch (currentState_) {
    case IDLE:
      if (event == Event::EMERGENCY) {
        setState(ERROR);
      } else {
        setState(RUNNING);
      }
      break;
    case RUNNING:
      setState(ERROR);
      break;
    case ERROR:
      /* Nothing for now */
      break;
  }
}

StateManager::RobotState StateManager::getCurrentState() const noexcept {
  std::lock_guard<std::mutex> lock(stateMutex_);
  return currentState_;
}

// PRIVATE API

std::string StateManager::eventToString(Event event) const noexcept {
  switch (event) {
    case Event::BATTERY_LOW:
      return "Battery Low";
    case Event::TEMPERATURE_HIGH:
      return "Temperature High";
    case Event::GPS_INACCURATE:
      return "GPS Inaccurate";
    case Event::SIGNAL_LOST:
      return "Signal Lost";
    case Event::SIGNAL_LOW:
      return "Signal Low";
    case Event::EMERGENCY:
      return "Emergency";
    default:
      return "Unknown event";
  }
}

void StateManager::setState(RobotState state) noexcept {
  currentState_ = state;
}

} // namespace gem_state_manager
